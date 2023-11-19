// Include classess and API (including LemLib used for Autonomous)
#include "main.h"
#include "api.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "lemlib/api.hpp"

//Temperary declarations

//Declare Controller
pros::Controller Master(pros::E_CONTROLLER_MASTER);

// Constant Values
const float WHEELDIAMETER = 4.125;
const float WHEELRADIUS = WHEELDIAMETER / 2;
const float PI = 3.14159;
const float DBRATIO = (double)72/48;

//Declare Inertial Sensor
pros::IMU Gyro(9);

//Declare Motors
pros::Motor MotorLeftF(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MotorLeftB(17, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MotorRightF(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MotorRightB(7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Intake(20, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Cata(10, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);

//Triport Objects
//Endgame/Hang
pros::ADIDigitalOut Blocker('B', false); //Piston starts extended through tubing
//AutonSelector
pros::ADIPotentiometer AutoSelector('C', pros::E_ADI_POT_V2);

//Declare Motor Groups
pros::Motor_Group LeftDB({MotorLeftF, MotorLeftB});
pros::Motor_Group RightDB({MotorRightF, MotorRightB});
pros::MotorGroup MotorGroupDriveBase({MotorLeftF, MotorLeftB, MotorRightF, MotorRightB});


//Use LemLib to create a DriveBase to allow for accurate autonomus
lemlib::Drivetrain_t drivetrain {
    &LeftDB, // left drivetrain motors
    &RightDB, // right drivetrain motors
    11.5625, // track width
    4, // wheel diameter
    300 // wheel rpm
};


//Sensor Declaration

lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    &Gyro // inertial sensor
};


//Methods for Auton and Driver control

//PID's

// Linear Movement PID (Forward and Reverse)
lemlib::ChassisController_t lateralController {
    1000, // kP
    0, // kD
    0, // smallErrorRange
    .1, // smallErrorTimeout
    10, // largeErrorRange
    10000, // largeErrorTimeout
    30 // slew rate
};
 
// Turning PID
lemlib::ChassisController_t angularController {
    8.5, // kP
    50, // kD
    .25, // smallErrorRange
    400, // smallErrorTimeout
    .5, // largeErrorRange
    1200, // largeErrorTimeout
    40 // slew rate
};

//Create Chassis (LemLib) For Auton Movement
lemlib::Chassis DB(drivetrain, lateralController, angularController, sensors);

double checkTime(double t){
	if(t < 0){
		return 0;
	}
	return t;
}

void badMovePID(double time, bool reverse){
	time *= 1000;
	double rev = 1;
	if(reverse){
		rev = -1;
	}
	MotorGroupDriveBase.move(127 * rev);
	pros::delay(checkTime(time - time / 5));
	MotorGroupDriveBase.move(60 * rev);
	pros::delay(time / 5);
}

void badTurnPID(double time, bool isRight){
	int direction = 0;
	if(isRight){
		direction = 1;
	} 
	else{
		direction = -1;
	}

	RightDB.move(127 * direction);
	LeftDB.move(127 * direction);
	pros::delay(checkTime(time - time / 5));
	RightDB.move(60 * direction);
	LeftDB.move(60 * direction);
	pros::delay(time / 5);
}


//Print all needed values
void screenPrint(){
	while(true){
		lemlib::Pose pose = DB.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
		pros::delay(20);
	}
}


//Adaptive new turning PID that utilizes degrees instead of turnTo()
void turn(double theta){
	theta = theta * -1 ;
	//Account for current degrees rotated
	theta += DB.getPose().theta;

	// Find coordinates accounting for current position
	double x = 1000 * (cos(theta) + DB.getPose().x);
  	double y = 1000 * (sin(theta) + DB.getPose().y);

	//Execution
	DB.turnTo(x, y, 1000);
	std::cout << "X: " << x << std::endl;
	std::cout << "Y: " << y << std::endl;
	std::cout << "Theta: " << theta << std::endl;
}

//Driver Control
void arcade(){
	//Declare variables for arcade drive
		double power = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		//Declare Arcade drive
		RightDB.move(power - turn);
		LeftDB.move(power + turn);

}

//Run Cata
// Spin catapult at volts, normally called for a period or upon button press
void cataLaunch(double volts){
	if(volts == 0){
		Cata.brake();
	}
	else{
		Cata.move(127);
	}
}

//Run Intake
// Spin intake at volts, most likely 127 or -127
void intake(double volts){
	if(volts == 0){
		Intake.brake();
	}
	else{
		Intake.move(volts);
	}
}

//Allow for deploy and redeploy of hang mech/endgame

//Allow endgame to be activated and deactivated
bool endgameFlag = true;
void blocker(){
	if(endgameFlag == true){
		Blocker.set_value(endgameFlag);
		endgameFlag = !endgameFlag;
	}
	else{
		endgameFlag = true;
		Blocker.set_value(false);
	}
}

//For selecting auton
bool right = false;
bool left = false;

void autoSelector(){
	//Show what auton was selected on the brain
	AutoSelector.calibrate();
	double current = AutoSelector.get_value_calibrated();
	std::string AutoSelection = "";
	while(pros::competition::is_disabled()){
		if(AutoSelector.get_value_calibrated() <= current + 90){
			AutoSelection = "Left";
			left = true;
			right = false;
		}
		else if(AutoSelector.get_value_calibrated() >= current + 240){
			AutoSelection = "Right";
			right = true;
			left = false;
		}
		else{
			AutoSelection = "N/A";
			left = false;
			right = false;
		}


		//Print the values to the brain screen
		pros::lcd::print(4,"Auton Selection: %s", AutoSelection);
		pros::lcd::print(5,"Potentiometer Reading: %f", AutoSelector.get_value_calibrated());

		pros::delay(50);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    //pros::lcd::initialize();
	//DB.calibrate();
	//DB.setPose(0,0,0);
	pros::lcd::initialize();
	AutoSelector.calibrate();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	//DB.setPose(0, 0, 0);
}

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
	autoSelector();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	pros::lcd::clear();
	pros::Task printTheScreen(screenPrint);
	//Auton
	//NOTE: Y is original lateral movement
	//NOTE: X is Perpendicular movement to placement
	//pros::Task printToScreen(screenPrint); -> used for testing only, prints screen during testing

	blocker();
	MotorGroupDriveBase.move(-127);
	pros::delay(1500);
	MotorGroupDriveBase.brake();
	/*LeftDB.move(100);
	RightDB.move(10);
	pros::delay(1000);
	MotorGroupDriveBase.brake();
	intake(127);
	MotorGroupDriveBase.move(127);
	pros::delay(4000);
	MotorGroupDriveBase.brake();
	MotorGroupDriveBase.move(-50);
	pros::delay(500);
	RightDB.move(60);
	LeftDB.move(-60);
	pros::delay(400);
	MotorGroupDriveBase.brake();
	intake(-80);
	RightDB.move(60);
	LeftDB.move(-60);
	pros::delay(400);
	MotorGroupDriveBase.move(-127);
	pros::delay(1500);
	MotorGroupDriveBase.brake();*/

	/*if(right){

	}
	if(left){
		MotorGroupDriveBase.move(-127);
		pros::delay(1500);
		MotorGroupDriveBase.brake();
	}*/
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

	//Print values on screen
	pros::Task printTheScreen(screenPrint);

	//User Control
	while(true){

		//Delay so information on screen is visible and calls are accurate
		pros::delay(20);

		//Allow for arcade drive 
		arcade();

		//Run intake
		if(Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){ //Intake
			intake(127);
		}
    	else if(Master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){ //Outtake
			intake(-127);
		}
    	else{ //Stop Intake	
        	intake(0);
        }
		
		//Run Cata
		if(Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			cataLaunch(127);
		}
		else{ //Stop Cata
			cataLaunch(0);
		}

		//Activate Endgame
		if(Master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			blocker();

		}
	}
}
