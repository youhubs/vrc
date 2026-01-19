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

// ===== PID TUNING CONSTANTS =====
// Centralized location for all PID gains - tune here instead of scattered throughout code
// HOW TO TUNE: See comments near lateral_controller and angular_controller definitions

// Lateral PID (forward/backward movement)
const double LATERAL_KP = 12.0;    // Proportional gain for driving
const double LATERAL_KI = 0.0;     // Integral gain (not typically needed)
const double LATERAL_KD = 47.0;    // Derivative gain for smooth driving

// Angular PID (turning)
const double ANGULAR_KP = 8.0;     // Proportional gain for turning
const double ANGULAR_KI = 0.0;     // Integral gain (not typically needed)
const double ANGULAR_KD = 67.67;   // Derivative gain for smooth turning

// Custom drive straight function PID
// TUNING GUIDE for driveStraight():
// Test: Call driveStraight(48, 50) and observe the path
// - If robot snakes/wiggles → decrease kP or increase kD
// - If robot drifts off straight → increase kP
// - If corrections feel jerky → increase kD
// Start with these values and adjust by ±50% if needed
const double DRIVE_STRAIGHT_KP = 0.1;   // Heading correction strength (typical range: 0.05-0.3)
const double DRIVE_STRAIGHT_KD = 0.05;  // Heading correction damping (typical range: 0.02-0.15)

// Custom turn function PID
const double TURN_KP = 0.5;   // Turn correction strength
const double TURN_KD = 0.1;   // Turn correction damping

// Driver control constants
const int JOYSTICK_DEADBAND = 5;  // Joystick values < 5 are treated as zero


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
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, -0.375);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller (forward/backward movement)
// HOW TO TUNE kP and kD:
// 1. Set kD=0, start with kP=1, test driving 48 inches
// 2. Increase kP until robot oscillates (wiggles left/right)
// 3. Reduce kP to 60% of oscillation point
// 4. Set kD = kP/10, gradually increase until smooth
// Symptoms:
//   - Overshoots target → increase kD or decrease kP
//   - Too slow/sluggish → increase kP
//   - Oscillates/wiggles → increase kD
//   - Never reaches target → increase kP
// NOTE: Values are defined at top of file in PID TUNING CONSTANTS section
lemlib::ControllerSettings lateral_controller(LATERAL_KP, // proportional gain (kP)
                                              LATERAL_KI, // integral gain (kI)
                                              LATERAL_KD, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              1, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller (turning)
// HOW TO TUNE kP and kD:
// 1. Set kD=0, start with kP=1, test turning 90°
// 2. Increase kP until robot oscillates back/forth
// 3. Reduce kP to 60% of oscillation point
// 4. Set kD = kP/10, gradually increase until smooth
// Symptoms:
//   - Overshoots angle → increase kD or decrease kP
//   - Turns too slowly → increase kP
//   - Oscillates back/forth → increase kD
//   - Never settles at target → increase kD
// NOTE: Values are defined at top of file in PID TUNING CONSTANTS section
lemlib::ControllerSettings angular_controller(ANGULAR_KP, // proportional gain (kP)
                                              ANGULAR_KI, // integral gain (kI)
                                              ANGULAR_KD, // derivative gain (kD)
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
    // CRITICAL: Robot must be stationary during calibration
    // 90% of mysterious failures come from bad calibration

    // IMU Sensor Reading Methods:
    // heading() - Compass mode: 0-360°, wraps around (359° → 0°)
    //             Use for: absolute direction checks (e.g., "facing the goal")
    // rotation() - Cumulative mode: unbounded, tracks total rotation
    //              Turn right twice = 720°, turn back left = value decreases
    //              Use for: PID control and turning logic (avoids wrap-around issues)
    // TIP: Default to rotation() for motion control to avoid "359° → 1°" problems

    pros::lcd::print(4, "Calibrating IMU...");
    chassis.calibrate();

    // Wait for calibration to finish
    // If the robot is moved/shaken during calibration, it will fail
    // and cause wild spinning or snake-like driving
    while (imu.is_calibrating()) {
        pros::delay(100);
    }
    pros::lcd::print(4, "IMU Calibrated!");

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

/**
 * Drive straight using gyro correction
 * Uses rotation() to avoid wrap-around issues with heading()
 *
 * @param dist Target distance in inches
 * @param speed Base speed (0-100 percent)
 * @param timeout Maximum time in milliseconds to prevent infinite loops
 */
void driveStraight(double dist, double speed, int timeout = 5000) {
    // Reset motor encoders
    left_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    right_motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    left_motors.tare_position();
    right_motors.tare_position();

    // Lock in target heading using rotation() (cumulative, avoids wrap-around)
    // Using rotation() instead of heading() prevents "359° → 1°" problems
    double targetHeading = imu.get_rotation();

    // PD gains for heading correction
    // NOTE: Values defined at top of file in PID TUNING CONSTANTS section
    // kP: Proportional gain - how aggressively to correct drift
    //     Higher = stronger correction but may cause side-to-side wiggling
    //     Lower = gentler correction but slower to fix drift
    // kD: Derivative gain - dampens oscillation for smoother driving
    //     Higher = less wiggling but may be sluggish to correct
    //     Adding kD prevents the "snake wiggle" problem common with P-only control
    double kP = DRIVE_STRAIGHT_KP;
    double kD = DRIVE_STRAIGHT_KD;

    double error = 0;
    double prevError = 0;

    // Convert distance from inches to encoder degrees
    // Wheel diameter affects this conversion
    double wheelCircumference = 2.75 * M_PI; // 2.75" wheel diameter
    double targetDegrees = (dist / wheelCircumference) * 360.0;

    // Start timer for timeout protection
    uint32_t startTime = pros::millis();

    // Loop until distance reached or timeout
    while (fabs(left_motors.get_position()) < fabs(targetDegrees)) {
        // Timeout check to prevent infinite loops
        if (pros::millis() - startTime > timeout) {
            break;
        }

        // --- Core PD Correction Logic ---
        // 1. How much have we drifted from target heading?
        double currentHeading = imu.get_rotation();
        error = targetHeading - currentHeading;

        // 2. Calculate correction strength using PD control
        // P term: proportional to current drift
        // D term: resists rapid changes, prevents wiggling/oscillation
        // Positive error (drifted left) → positive correction
        // Negative error (drifted right) → negative correction
        double derivative = error - prevError;
        double correction = error * kP + derivative * kD;

        // 3. Apply correction to motors
        // If drifted left (positive error): slow left motor, speed up right motor
        // If drifted right (negative error): speed up left motor, slow right motor
        left_motors.move(speed - correction);
        right_motors.move(speed + correction);

        prevError = error; // Store error for next derivative calculation

        // Small delay to prevent CPU overload
        pros::delay(20);
    }

    // Stop motors when target reached
    left_motors.brake();
    right_motors.brake();
}

/**
 * Turn to target angle using PD control
 * Uses rotation() for cumulative angle tracking (avoids wrap-around)
 *
 * @param targetAngle Target angle in degrees (can be >360 or negative)
 * @param timeout Maximum time in milliseconds to prevent infinite loops
 * @param tolerance Acceptable error in degrees (default 1.0°)
 */
void turnPID(double targetAngle, int timeout = 3000, double tolerance = 1.0) {
    // PD gains
    // NOTE: Values defined at top of file in PID TUNING CONSTANTS section
    // kP: Proportional gain - how aggressively to correct error
    //     Higher = faster turning but may overshoot/oscillate
    // kD: Derivative gain - dampens oscillation by resisting rapid changes
    //     Higher = smoother but slower settling
    double kP = TURN_KP;
    double kD = TURN_KD;

    double error = 0;
    double prevError = 0;

    // Start timer for timeout protection
    uint32_t startTime = pros::millis();

    // Settling time tracking - ensures robot is stable at target
    int settledCount = 0;
    const int requiredSettledCycles = 3; // Must be within tolerance for 3 cycles

    while (true) {
        // Timeout check to prevent infinite loops
        if (pros::millis() - startTime > timeout) {
            break;
        }

        // Use rotation() instead of heading() to avoid wrap-around issues
        // This allows turning to angles like 720° or -180°
        double currentAngle = imu.get_rotation();
        error = targetAngle - currentAngle;

        // Exit condition: error is within tolerance AND robot has settled
        if (fabs(error) < tolerance) {
            settledCount++;
            if (settledCount >= requiredSettledCycles) {
                break; // Successfully reached target
            }
        } else {
            settledCount = 0; // Reset if we drift outside tolerance
        }

        // --- PD Control Calculation ---
        // P term: proportional to current error
        // D term: proportional to rate of change (prevents overshoot)
        double derivative = error - prevError;
        double speed = error * kP + derivative * kD;

        // Clamp speed to prevent excessive motor strain
        // Maximum turn speed: 80% to maintain control
        if (speed > 80) speed = 80;
        if (speed < -80) speed = -80;

        // In-place turning: left and right motors spin opposite directions
        // Positive error (need to turn right): left forward, right backward
        // Negative error (need to turn left): left backward, right forward
        left_motors.move(speed);
        right_motors.move(-speed);

        prevError = error;
        pros::delay(20); // Small delay to prevent CPU overload
    }

    // Stop motors when target reached or timeout
    left_motors.brake();
    right_motors.brake();
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

        // Apply deadband to filter out joystick jitter
        // V5 joysticks never truly return to zero and may jitter between -5 and 5
        // NOTE: Deadband value defined at top of file in PID TUNING CONSTANTS section
        if (abs(turn) < JOYSTICK_DEADBAND) {
            turn = 0;
        }
        if (abs(forward) < JOYSTICK_DEADBAND) {
            forward = 0;
        }

        // Apply exponential mapping for better low-speed precision
        // Linear mapping (50% = 50%) is intuitive but lacks precision
        // Exponential curve: low speeds become ultra-precise, high speeds unchanged
        // Formula: Output = (Input^3) / 10000
        turn = (turn * turn * turn) / 10000;
        forward = (forward * forward * forward) / 10000;

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