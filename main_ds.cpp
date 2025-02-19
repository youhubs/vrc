#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstdlib>

// =============================================
// Device Configuration
// =============================================
namespace Devices {
    // Drivetrain motors
    pros::MotorGroup left_motors({10, -7, -5}, pros::MotorGear::blue);
    pros::MotorGroup right_motors({-9, 1, 6}, pros::MotorGear::blue);
    
    // Subsystem motors
    pros::Motor preRoller(-3);
    pros::Motor conv(4, pros::MotorGear::blue, pros::MotorUnits::counts);
    pros::Motor lb(-8);
    
    // Sensors
    pros::Optical ring(11);
    pros::Imu imu(12);
    
    // Pneumatics
    pros::adi::DigitalOut mogo('A');
    pros::adi::DigitalOut leftDoinker('B');
    pros::adi::DigitalOut rightDoinker('H');
}

// =============================================
// Global State Management
// =============================================
namespace State {
    // Lift control states
    const int NUM_STATES = 3;
    const int LIFT_STATES[NUM_STATES] = {0, 230, 1800};  // Encoder positions
    int current_lift_state = 0;
    int lift_target = 0;
    
    // Toggle controls
    struct Toggle {
        bool state = false;
        bool latch = false;
    };
    
    Toggle mogo_clamp;
    Toggle corner_doinker;
    Toggle side_doinker;
    
    // Conveyor control
    bool conveyor_active = false;
}

// =============================================
// Chassis Configuration
// =============================================
lemlib::Chassis createChassis() {
    // Drivetrain configuration
    lemlib::Drivetrain drivetrain(
        &Devices::left_motors,
        &Devices::right_motors,
        11.875,    // Track width
        2.75,      // Wheel diameter
        450,       // RPM
        2          // Chase power
    );

    // Sensor configuration
    lemlib::OdomSensors sensors(
        nullptr, nullptr, nullptr, nullptr, &Devices::imu
    );

    // PID controllers
    lemlib::ControllerSettings lateral_controller(
        30, 0, 120,   // PID gains
        3,             // Anti-windup
        0.1, 100,      // Small error thresholds
        0.5, 500,      // Large error thresholds
        30             // Max acceleration
    );

    lemlib::ControllerSettings angular_controller(
        6, 0, 38,     // PID gains
        3,             // Anti-windup
        1, 100,        // Small error thresholds
        3, 500,        // Large error thresholds
        0              // Max acceleration
    );

    return lemlib::Chassis(
        drivetrain,
        lateral_controller,
        angular_controller,
        sensors,
        nullptr,
        nullptr
    );
}

lemlib::Chassis chassis = createChassis();

// =============================================
// Subsystem Control Functions
// =============================================
void updateLiftState() {
    State::current_lift_state += 1;
    if (State::current_lift_state == State::NUM_STATES) {
        State::current_lift_state = 0;
    }
    State::lift_target = State::LIFT_STATES[State::current_lift_state];
}

void liftControl() {
    constexpr double kP = 0.5;
    constexpr double DEADBAND = 2.0;
    
    double error = State::lift_target - Devices::lb.get_position();
    double velocity = kP * error;

    // Apply deadband
    if (std::abs(error) < DEADBAND) velocity = 0;
    
    Devices::lb.move(velocity);
}

void checkColorAndSort() {
    auto rgb = Devices::ring.get_rgb();
    if (rgb.blue >= rgb.red) {
        pros::delay(160);
        Devices::conv.move(0);
        pros::delay(150);
    }
}

void conveyorControl(bool run) {
    if (run) {
        Devices::conv.move(127);
        checkColorAndSort();
    } else {
        Devices::conv.move(0);
    }
}

// =============================================
// Autonomous & Driver Control Core Functions
// =============================================
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    chassis.setPose(0, 1, 0);
    Devices::lb.tare_position();

    // Screen update task
    pros::Task screen_task([] {
        while (true) {
            auto pose = chassis.getPose();
            auto rgb = Devices::ring.get_rgb();
            
            pros::lcd::print(0, "X: %.1f Y: %.1f", pose.x, pose.y);
            pros::lcd::print(1, "Heading: %.1f°", pose.theta);
            pros::lcd::print(2, "RGB: %d,%d,%d", rgb.red, rgb.green, rgb.blue);
            pros::delay(20);
        }
    });

    // Lift control task
    pros::Task lift_task([] {
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });
}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    Devices::lb.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    // Initial conveyor operation
    conveyorControl(true);
    
    // Example autonomous routine (keep your existing sequence here)
    // chassis.moveToPoint(0, 24, 2000);
    // chassis.turnToHeading(90, 1000);
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    Devices::preRoller.set_brake_mode(pros::MotorBrake::coast);

    while (true) {
        // Tank drive control
        int power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        Devices::left_motors.move(power + turn);
        Devices::right_motors.move(power - turn);

        // Intake controls
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            conveyorControl(true);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            Devices::conv.move(-200);
        } else {
            conveyorControl(false);
        }

        // Roller controls
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            Devices::preRoller.move(200);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {  
            Devices::preRoller.move(-200);
        } else {
            Devices::preRoller.move(0);
        }

        // Lift state control
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            updateLiftState();
            if (State::current_lift_state == 2) {
                Devices::conv.move(-80);
                pros::delay(200);
                Devices::conv.move(0);
            }
        }

        // Pneumatic controls
        auto handleToggle = [&](State::Toggle& toggle, pros::controller_digital_e_t button) {
            if (master.get_digital(button)) {
                if (!toggle.latch) {
                    toggle.state = !toggle.state;
                    toggle.latch = true;
                }
            } else {
                toggle.latch = false;
            }
            return toggle.state;
        };

        Devices::mogo.set_value(handleToggle(State::mogo_clamp, pros::E_CONTROLLER_DIGITAL_X));
        Devices::leftDoinker.set_value(handleToggle(State::corner_doinker, pros::E_CONTROLLER_DIGITAL_B));
        Devices::rightDoinker.set_value(handleToggle(State::side_doinker, pros::E_CONTROLLER_DIGITAL_A));

        pros::delay(20);
    }
}

// =============================================
// Competition Required Functions
// =============================================
void disabled() {}
void competition_initialize() {}