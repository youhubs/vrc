#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include <cstdlib>


pros::MotorGroup left_motors({10, -7, -5}, pros::MotorGear::blue);
pros::MotorGroup right_motors({-9, 1, 6}, pros::MotorGear::blue);
pros::Motor preRoller(-3);
pros::Motor conv(4, pros::MotorGear::blue, pros::MotorUnits::counts);
pros::Motor lb(-8);

pros::Optical ring(11);

// Add inertial sensor(imu) and ration sensor
pros::Imu imu(9);
pros::Rotation horizontalRot(5);

const int numStates = 3;
// Target positions in motor encoder ticks (assuming 1 degree = 5 ticks; adjust as needed)
int states[numStates] = {0, 230, 1800};
int currState = 0;
int target = 0;

void nextState() {
    currState += 1;
    if (currState == numStates) {
        currState = 0;
    }
    target = states[currState];
}

void liftControl() {
    double kp = 0.5;
    double error = target - lb.get_position(); // Use motor's internal encoder
    double velocity = kp * error;

    // Prevent overshooting or oscillations with clamping
    if (std::abs(error) < 2) { // Deadband for small errors (adjust if needed)
        velocity = 0;
    }

    lb.move(velocity);
}


pros::adi::DigitalOut mogo('A');
pros::adi::DigitalOut leftDoinker('B');
pros::adi::DigitalOut rightDoinker('H');

bool mToggle = false;

bool mLatch = false;
// Toggle and Latch for Corner Clear
bool cLatch = false;

bool cToggle = false;

bool dLatch = false;

bool dToggle = false;


lemlib:: Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    11.875,
    2.75,
    450,
    2
);

lemlib::TrackingWheel horizontalWheel(
    &horizontalRot, 
    2,    // Wheel diameter (inches)
    -0.25,     // Distance from center (negative = left side, positive = right)
    1.0       // Gear ratio (1:1 if directly connected)
);

// stating the odom
lemlib::OdomSensors sensors (
    nullptr, //&vertical_tracking_wheel,
    nullptr,
    &horizontalWheel, //&horizontal_tracking_wheel,
    nullptr,
    &imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(30   , // proportional gain (kP)
                                            0, // integral gain (kI)
                                            120, // derivative gain (kD)
                                            3, // anti windup
                                            .1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            .5, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            30 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(6, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              38, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);


// input curve for throttle input during driver control
/*lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
*/
// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        nullptr, 
                        nullptr
);


void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    horizontalRot.reset();   // Reset horizontal tracking wheel
    chassis.calibrate();     // calibrate sensors
    chassis.setPose(0, 1, 0);
    lb.tare_position();
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Heading|Theta: %f", chassis.getPose().theta); // heading

            double horizontalDisplacement = horizontalRot.get_position();
            pros::lcd::print(3, "Horizontal: %f", horizontalDisplacement);

            // delay to save resources
            pros::delay(20);
        }
    });
    
    //lb.tare_position();

    // Create a task to continuously control the lift motor
    pros::Task liftControlTask([] {
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });
    
}

void moveConveyerTask() {
    while (true) {
        pros::c::optical_rgb_s_t rgb_value = ring.get_rgb();
        if (rgb_value.blue >= rgb_value.red) {
            pros::delay(160);
            conv.move(0);
            pros::delay(150);
            conv.move(127);
        }
        pros::delay(20); // Prevent CPU overload
    }
}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    lb.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    
    pros::Task conveyorTask(moveConveyerTask);
    /*
    chassis.moveToPose(24,24,90, 2000,{.maxSpeed = 127});
    chassis.turnToHeading(180,2000);
    chassis.moveToPose(0,0,270, 2000);
    chassis.turnToHeading(0,2000);
    chassis.moveToPose(0,0,0,2000);
    // chassis.turnToHeading(90,1000);
    */

    //grab mogo
    chassis.moveToPose(0, -32, 0, 1100, {.forwards = false, .maxSpeed = 80});
    pros::delay(875);
    mogo.set_value(true);
    //turn to middle rings
    chassis.turnToHeading(-210, 600, {.maxSpeed = 120});
    //pick up middle rings
    pros::delay(100);
    chassis.moveToPose(20, -40, -270, 1250, {.forwards = true, .maxSpeed = 80});
    pros::delay(300);
    preRoller.move(127);
    conv.move(127);
    chassis.moveToPose(30, -40, -270, 1100, {.forwards = true, .maxSpeed = 90});
    //move to the third ring
    chassis.moveToPose(24, -25, 315, 1500, {.maxSpeed = 90});
    pros::delay(250);
    //rings in mid
    //chassis.moveToPose(-22, -3, -90, 1600, {.maxSpeed = 110});
    
    //chassis.moveToPose(-38, -3, -90, 1200, {.maxSpeed = 80});
    pros::delay(100);
    /*
    chassis.turnToHeading(50, 750);
    pros::delay(300);
    conv.move(0);
    chassis.moveToPose(-29, 4, 90, 1500, {.maxSpeed = 100});
    preRoller.move(0);
    chassis.turnToHeading(175, 750);
    chassis.moveToPoint(-30, 12, 1000, {.forwards = false});
    pros::delay(500);
    conv.move(127);
    pros::delay(1000);
    conv.move(0);
    
    chassis.moveToPoint(-29 , -28, 2000, {.maxSpeed = 120});

    */
    
    //chassis.moveToPoint(22, -50, 1500, {.forwards = false, .maxSpeed = 90});
    

    //8.5
    /*pros::delay(100);
    chassis.turnToHeading(45, 700)
    pros::delay(100);
    chassis.moveToPoint(24, -30, 1500, {.maxSpeed = 90});
    pros::delay(100);
    chassis.turnToHeading(-250, 800);
    pros::delay(300);
    preRoller.move(0);
    chassis.moveToPose(-20 ,-17,270, 3700 , {.forwards = false, .maxSpeed = 127});
    pros::delay(3000);
    mogo.set_value(false);
    chassis.turnToHeading(185,1000);
    pros::delay(100);
    chassis.moveToPoint(-24, -9, 400, {.forwards = false, .maxSpeed = 80});
    pros::delay(100);
    //chassis.moveToPoint(27, -11, 400);
    
    pros::delay(350);
    preRoller.move(200);
    */
    
    //chassis.moveToPoint(26, 4, 500, {.forwards = false});


    //chassis.moveToPoint(,-34,2000);

    pros::delay(200);
    
    //chassis.turnToHeading(-315, 1500);
    //chassis.moveToPoint(0, -33,  2000, {.forwards = true, .maxSpeed = 30 });

}


void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

    preRoller.set_brake_mode(pros::MotorBrake::coast);


 while (true) {
    int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    left_motors.move(left);
    right_motors.move(right);
    
        // Tank drive control
        pros::c::optical_rgb_s_t rgb_value;
        rgb_value = ring.get_rgb();
        
        // Intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            conv.move(200); // Spin intake forward
            if(rgb_value.blue >= rgb_value.red*1.5) {
                pros::delay(160);
                conv.move(0);
                pros::delay(150);
            }
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            conv.move(-200); // Spin intake backward
        } else {
            conv.move(0); // Stop intake    
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            preRoller.move(200); // Spin intake forward
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {  
            preRoller.move(-200); // Spin intake backward
        } else {
            preRoller.move(0); // Stop intake
        }
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            nextState();
            if(currState == 2) {
                conv.move(-80);
                pros::delay(200);
                conv.move(0);
            }
        }


        mogo.set_value(mToggle);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            if(!mLatch){
                mToggle = !mToggle;
                mLatch = true;
            }
        } else {
            mLatch = false;
        }

        
        leftDoinker.set_value(cToggle);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            if(!cLatch){
                cToggle = !cToggle;
                cLatch = true;
            }
        } else {
            cLatch = false; 
        }
        rightDoinker.set_value(dToggle);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            if(!dLatch){
                dToggle = !dToggle;
                dLatch = true;
            }
        } else {
            dLatch = false; 
        }
        
        pros::lcd::print(2, "imu: %f", imu.get_heading());        
        pros::lcd::print(3, "imu: %f", rgb_value.red);        
        pros::lcd::print(4, "imu: %f", rgb_value.green);        
        pros::lcd::print(5, "imu: %f", rgb_value.blue);        
        
        pros::delay(20); // Short delay to prevent CPU overload
    }


}