#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstdlib>



pros::MotorGroup left_motors({10, -7, -5}, pros::MotorGear::blue);
pros::MotorGroup right_motors({-9, 1, 6}, pros::MotorGear::blue);
pros::Motor preRoller(-3);
pros::Motor conv(4, pros::MotorGear::blue, pros::MotorUnits::counts);
pros::Motor lb(-8);



pros::Optical ring(11);

/*
const int numStates = 3;
int states[numStates] = {0, 300, 2000};
int
*/

const int numStates = 3;
// Target positions in motor encoder ticks (assuming 1 degree = 5 ticks; adjust as needed)
int states[numStates] = {0, 230, 1800};
int currState = 0;
int target = 0;
bool moveConv = false;

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

pros::Task colorSort() {
    pros::c::optical_rgb_s_t rgb_value;
    rgb_value = ring.get_rgb();
    if(rgb_value.blue >= rgb_value.red) {
        pros::delay(160);
        conv.move(0);
        pros::delay(150);
    }
}

pros::Task moveConveyer() {
    
    conv.move(127);
    colorSort();

}


pros::Task stopConveyer() {
    conv.move(0);
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


// initialization of inertial sensor(imu)
pros::Imu imu(12);



// stating the odom
lemlib::OdomSensors sensors (
	nullptr, //&vertical_tracking_wheel,
	nullptr,
	nullptr, //&horizontal_tracking_wheel,
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
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0, 1, 0);
    lb.tare_position();
    pros::c::optical_rgb_s_t rgb_value;
    rgb_value = ring.get_rgb();
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Heading|Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "imu: %f", rgb_value.red);        
            pros::lcd::print(4, "imu: %f", rgb_value.green);        
            pros::lcd::print(5, "imu: %f", rgb_value.blue); 
            // heading
            // delay to save resources
            pros::delay(20);
        }
    });
    

    //lb.tare_position();    // Create a task to continuously control the lift motor
    pros::Task liftControlTask([] {
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });
   
    
}




void disabled() {}
void competition_initialize() {}

void autonomous() {
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    lb.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    /*
    chassis.moveToPose(24,24,90, 2000,{.maxSpeed = 127});
    chassis.turnToHeading(180,2000);
    chassis.moveToPose(0,0,270, 2000);
    chassis.turnToHeading(0,2000);
    chassis.moveToPose(0,0,0,2000);
    // chassis.turnToHeading(90,1000);
    */
    pros::c::optical_rgb_s_t rgb_value;
    rgb_value = ring.get_rgb();
    conv.move(127);
    if(rgb_value.blue >= rgb_value.red) {
        pros::delay(160);
        conv.move(0);
        pros::delay(150);
    }

    //chassis.moveToPoint(0, 24, 2000);
   

    /*nextState(); 
    nextState();
    pros::delay(600);
    nextState();
    chassis.moveToPoint(0, -37, 1750, {.forwards = false, .maxSpeed = 85});
    pros::delay(150);
    chassis.turnToHeading(-90, 800);
    chassis.moveToPoint(35, -21.5, 1500, {.forwards = false, .maxSpeed = 75});
    pros::delay(1100);
    //mogo.set_value(true);
    chassis.turnToHeading(-305, 1000);
    chassis.moveToPoint(47, -12, 5000, {.maxSpeed = 80});
    pros::delay(1200);
    rightDoinker.set_value(true);
    pros::delay(200);
    //chassis.turnToHeading(45, 500);
    chassis.swingToHeading(85, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed =70});
    pros::delay(900);
    leftDoinker.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(2, -36,  4000, {.forwards = false, .maxSpeed = 90});
    pros::delay(2000);
    leftDoinker.set_value(false);
    rightDoinker.set_value(false);
    //chassis.moveToPoint(,-34,2000);s
    */


    /*chassis.turnToHeading(55, 600, {.maxSpeed = 90});
    chassis.moveToPoint(-3, -51, 3000, { .forwards = false, .maxSpeed = 90});
    pros::delay(2000);
    leftDoinker.set_value(false);
    rightDoinker.set_value(false);
    moveConveyer();
    preRoller.move(127);
    chassis.swingToHeading(10, lemlib::DriveSide::LEFT, 600);
    pros::delay(200);
    chassis.swingToHeading(160, lemlib::DriveSide::RIGHT, 1200);
    chassis.moveToPose(18, -90, 188, 1000);
*/
    
    

}


void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

	preRoller.set_brake_mode(pros::MotorBrake::coast);




	
	
 while (true) {

    int power = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    int left = power + turn;
    int right = power - turn;


    left_motors.move(left);
    right_motors.move(right);
    
        // Tank drive control
    
        pros::c::optical_rgb_s_t rgb_value;
        rgb_value = ring.get_rgb();
        
        // Intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
             // Spin intake forward
             conv.move(127);
             colorSort();
            
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
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            if(!cLatch){
                cToggle = !cToggle;
                cLatch = true;
            }
        } else {
            cLatch = false; 
        }
        rightDoinker.set_value(dToggle);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            if(!dLatch){
                dToggle = !dToggle;
                dLatch = true;
            }
        } else {
            dLatch = false; 
        }
        
        
        pros::lcd::print(2, "imu: %f", imu.get_heading());        
               
                
        
        pros::delay(20); // Short delay to prevent CPU overload
    }



    
	

}






