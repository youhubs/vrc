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
    pros::MotorGroup left_motors({18, -17, -15}, pros::MotorGear::blue);
    pros::MotorGroup right_motors({-13, 16, 19}, pros::MotorGear::blue);
    
    // Subsystem motors
    pros::Motor preRoller(-12);
    pros::Motor conv(11, pros::MotorGear::blue, pros::MotorUnits::counts);
    pros::Motor lb(-1);
    
    // Sensors
    pros::Optical ring(10);
    pros::Imu imu(12);

    // New tracking wheel components
    pros::Rotation horizontal_encoder(20);
    
    // Pneumatics
    pros::adi::DigitalOut mogo('B');
    pros::adi::DigitalOut leftDoinker('C');
    pros::adi::DigitalOut rightDoinker('A');
}

// =============================================
// Global State Management
// =============================================
namespace State {
    // Lift control states
    const int NUM_STATES = 3;
    const int LIFT_STATES[NUM_STATES] = {0, 230, 1800};
    int current_lift_state = 0;
    int lift_target = 0;

    // Toggle control structure
    struct Toggle {
        bool state = false;
        bool latch = false;
    };
    
    Toggle mogo_clamp;
    Toggle corner_doinker;
    Toggle side_doinker;
    
    // Conveyor control state
    bool conveyor_active = false;
}

namespace Tracking {
    lemlib::TrackingWheel horizontal_wheel(
        &Devices::horizontal_encoder, 
        2.75,    // Wheel diameter (inches)
        -3.25,   // Distance from center (negative = left side, positive = right)
        1.0      // Gear ratio (1:1 if directly connected)
    );
}

// =============================================
// Chassis Configuration
// =============================================
lemlib::Chassis createChassis() {
    // Drivetrain configuration
    lemlib::Drivetrain drivetrain(
        &Devices::left_motors,
        &Devices::right_motors,
        11.875,       // Track width
        2.75,         // Wheel diameter
        450,          // RPM
        2             // Chase power
    );

    // Sensor setup
    lemlib::OdomSensors sensors(
        nullptr,                     // Vertical tracking wheel (none)
        &Tracking::horizontal_wheel, // Horizontal tracking wheel
        nullptr,                     // Second horizontal wheel (none)
        nullptr,                     // Back tracking wheel (none)
        &Devices::imu                // IMU
    );

    // PID controllers
    lemlib::ControllerSettings lateral_controller(
        30, 0, 120,       // kP/kI/kD
        3,                // Anti-windup
        0.1, 100,         // Small error thresholds
        0.5, 500,         // Large error thresholds
        30                // Max acceleration
    );

    lemlib::ControllerSettings angular_controller(
        6, 0, 38,         // kP/kI/kD
        3,                // Anti-windup
        1, 100,           // Small error thresholds
        3, 500,           // Large error thresholds
        0                 // Max acceleration
    );

    return lemlib::Chassis(
        drivetrain,
        lateral_controller,
        angular_controller,
        sensors,
        nullptr,          // Drive curve (left)
        nullptr           // Drive curve (right)
    );
}

// Global chassis instance
lemlib::Chassis chassis = createChassis();

// =============================================
// Subsystem Control
// =============================================
void updateLiftState() {
    State::current_lift_state = (State::current_lift_state + 1) % State::NUM_STATES;
    State::lift_target = State::LIFT_STATES[State::current_lift_state];
}

void liftControl() {
    constexpr double kP = 0.5;
    constexpr double DEADBAND = 2.0;
    
    double error = State::lift_target - Devices::lb.get_position();
    double velocity = kP * error;

    // Apply deadband to prevent oscillations
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
// Core System Functions
// =============================================
void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();
    chassis.setPose(0, 1, 0);
    Devices::lb.tare_position();
    Devices::horizontal_encoder.reset_position();
    Devices::conv.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // Screen update task
    pros::Task screen_task([] {
        while (true) {
            auto pose = chassis.getPose();
            auto rgb = Devices::ring.get_rgb();
            
            pros::lcd::print(0, "X: %f", pose.x);
            pros::lcd::print(1, "Y: %f", pose.y);
            pros::lcd::print(2, "Heading|Theta: %f", pose.theta);
            pros::lcd::print(3, "R: %f", rgb.red);
            pros::lcd::print(4, "G: %f", rgb.green);
            pros::lcd::print(5, "B: %f", rgb.blue);
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

    // chassis.moveToPose(24,24,90, 2000,{.maxSpeed = 127});
    // chassis.turnToHeading(180,2000);
    // chassis.moveToPose(0,0,270, 2000);
    // chassis.turnToHeading(0,2000);
    // chassis.moveToPose(0,0,0,2000);
    // chassis.turnToHeading(90,1000);

    // Start conveyor task
    pros::Task conveyor_task([] {
        while (true) {
            conveyorControl(true);
            pros::delay(20);
        }
    });

    updateLiftState(); 
    updateLiftState();
    pros::delay(600);
    updateLiftState();
    chassis.moveToPoint(0, -37, 1750, {.forwards = false, .maxSpeed = 85});
    pros::delay(150);
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(-35, -21.5, 1500, {.forwards = false, .maxSpeed = 75});

    pros::delay(1100);
    Devices::mogo.set_value(true);
    chassis.turnToHeading(305, 1000);
    chassis.moveToPoint(-42, -14, 5000, {.maxSpeed = 80});
    pros::delay(1050);
    Devices::leftDoinker.set_value(true);
    pros::delay(200);
    //chassis.turnToHeading(45, 500);
    chassis.swingToHeading(-78, lemlib::DriveSide::LEFT, 1000, {.maxSpeed =70});
    pros::delay(800);
    Devices::rightDoinker.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(4, -39,  4000, {.forwards = false, .maxSpeed = 90});
    pros::delay(1750);
    Devices::leftDoinker.set_value(false);
    Devices::rightDoinker.set_value(false);
    //chassis.moveToPoint(,-34,2000);s
    
    Devices::conv.move(127);
    Devices::preRoller.move(127);
    chassis.swingToHeading(-20, lemlib::DriveSide::LEFT, 900, {.maxSpeed = 90});
    pros::delay(200);
    chassis.swingToHeading(-165, lemlib::DriveSide::RIGHT, 1200, {.maxSpeed = 80});
    chassis.moveToPose(-30, -66, 135, 1000);
}


void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
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
        Devices::leftDoinker.set_value(handleToggle(State::corner_doinker, pros::E_CONTROLLER_DIGITAL_A));
        Devices::rightDoinker.set_value(handleToggle(State::side_doinker, pros::E_CONTROLLER_DIGITAL_B));

        pros::delay(20);
    }
}

// =============================================
// Competition Required Functions
// =============================================
void disabled() {}
void competition_initialize() {}