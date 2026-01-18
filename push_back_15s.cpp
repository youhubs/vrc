#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <functional>


//motors
pros::MotorGroup left_motors({-1, -11, -13}, pros::MotorGearset::blue); // left motors on ports 11, 12, 1
pros::MotorGroup right_motors({10, 20, 18}, pros::MotorGearset::blue); // right motors on ports 20, 19, 10
pros::Motor Front_Intake(9, pros::MotorGearset::blue);
pros::Motor Back_Intake(2, pros::MotorGearset::blue);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Pneumatics
pros::adi::Pneumatics Middle_Goal('A',false);
pros::adi::Pneumatics Wing('C',false);
pros::adi::Pneumatics Scraper('D',false);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              11.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2 (for now)
);


//Horizontal -3.75 tracking wheel offset
//Vertical -2 tracking wheel offset
pros::Imu imu(4);
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(17);
// vertical tracking wheel (offset in inches)
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -2.0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(12, // proportional gain (kP) 13
                                              0, // integral gain (kI)
                                              47, // derivative gain (kD) 60.8
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(8, // proportional gain (kP)8
                                              0, // integral gain (kI)
                                              67.67, // derivative gain (kD)67
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              1, // large error range, in degrees
                                              100, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, //Lateral PID settings 
                        angular_controller, //Angular PID settings
                        sensors // odometry sensors
                        
);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button() {
    static bool pressed = false;
    pressed = !pressed;
    if (pressed) {
        pros::lcd::set_text(2, "I was pressed!");
    } else {
        pros::lcd::clear_line(2);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();

    // Calibrate IMU and reset odometry
    chassis.calibrate();
    chassis.setPose(0, 0, 90);
    vertical_encoder.reset();
    vertical_encoder.set_reversed(true);

    // Background task to print pose and sensor values
    pros::Task([]() {
        while (true) {
            pros::lcd::print(0, "X: %.2f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %.2f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %.2f", chassis.getPose().theta);
            pros::lcd::print(3, "IMU: %.2f", imu.get_heading());
            pros::lcd::print(5, "V Enc: %d", vertical_encoder.get_position());
            pros::delay(50);
            
        }
    });
}

void disabled() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */


/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {

}


void autonomous() {
    //Start
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(0, 0, 90);
    chassis.turnToPoint(33,0,1999);
    chassis.moveToPoint(31.5,0,900,{.maxSpeed = 110});
    Scraper.extend();
    //Turn to loader and Load
    chassis.turnToPoint(31.5,-9,710,{.maxSpeed=75});
    chassis.moveToPoint(31.5,-9,850,{.maxSpeed=45});
    Front_Intake.move(127);
    Back_Intake.move(45);
    pros::delay(1050);
    //Go To Top Goal and Score
    chassis.moveToPoint(33,20,1000,{.forwards=false,.maxSpeed=120});
    pros::delay(800);
    Back_Intake.move(-127);
    pros::delay(1170);
    Back_Intake.move(127);
    Scraper.retract();
    chassis.moveToPoint(33,8,670);
    chassis.turnToPoint(8,24.5,700);
    Back_Intake.move(127);
    chassis.moveToPoint(8,24.5,600);
    pros::delay(400);
    Scraper.extend();
    pros::delay(100);
    chassis.turnToPoint(-40.58,24.5,600);
    pros::delay(250);
    Scraper.retract();
    chassis.moveToPoint(-40.58,24.5,1100);
    pros::delay(640);
    Scraper.extend();
    chassis.turnToPoint(-28.48,37.6,400,{.forwards=false});
    chassis.moveToPoint(-28.48,37.6,700,{.forwards=false});
    pros::delay(650);
    Middle_Goal.extend();
    Front_Intake.move(-127);
    Back_Intake.move(127);
    pros::delay(167);
    Front_Intake.move(127);
    pros::delay(900);
    Middle_Goal.retract();
    chassis.turnToPoint(-69.5,0,200);
    chassis.moveToPoint(-69.5,0,1300);
    Scraper.extend();
    chassis.turnToPoint(-69.5,-13.5,450);
    Back_Intake.move(127);
    chassis.moveToPoint(-69.5,-17,1100, {.maxSpeed=60});
    pros::delay(1000);
    chassis.moveToPoint(-69.5,18,1300,{.forwards=false});
    pros::delay(600);
    Back_Intake.move(-127);

    // Stop all motors at end of autonomous
    Front_Intake.move(0);
    Back_Intake.move(0);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    // loop forever
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    while (true) {
        
        // get left y and right x positions
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int forward = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.arcade(forward, turn);

        // delay to save resources
        pros::delay(25);

        // Intake and Outtake control
        // middle goal score
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            Middle_Goal.extend();
            Front_Intake.move(127);
            Back_Intake.move(67);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // bottom goal
            Front_Intake.move(-127);
            Back_Intake.move(127); 

        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // intake
            Front_Intake.move(127);
            Back_Intake.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // top goal
            Front_Intake.move(127);
            Back_Intake.move(-127);
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){ // matchload
            Scraper.toggle();

        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){ // wing
            Wing.toggle();

        } else {
            Front_Intake.move(0);
            Back_Intake.move(0);
            Middle_Goal.retract();
        } 
    }
}