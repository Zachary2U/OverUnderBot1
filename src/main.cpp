// Include classess and API
#include "main.h"
#include "api.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "lemlib/api.hpp"
//#include "classess.cpp"

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
pros::ADIDigitalOut Endgame('B', false); //Piston starts extended through tubing

//Declare Motor Groups
pros::Motor_Group LeftDB({MotorLeftF, MotorLeftB});
pros::Motor_Group RightDB({MotorRightF, MotorRightB});


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
    8, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    5 // slew rate
};
 
// Turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    0 // slew rate
};

//Create Chassis (LemLib) For Auton Movement
lemlib::Chassis DB(drivetrain, lateralController, angularController, sensors);

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

/**void pidMove(double dist){
  double volts, pastError, derivative;
  double kP = 4.20;
  double kD = 0;
  LeftDB.set_zero_position(0);
  RightDB.set_zero_position(0);
  double initialDist = ((RightDB.get_positions()[0] + LeftDB.get_positions()[0]) / 2) * DBRATIO * WHEELRADIUS * PI;
  double error = dist;
  while(fabs(error) > 0.3){
    double reading = (((RightDB.get_positions()[0] + LeftDB.get_positions()[0]) / 2) * DBRATIO * WHEELRADIUS * PI) - initialDist;
    error = dist - reading;
    derivative = (error - pastError);
    volts = (error * kP) - (derivative * kD);
    LeftDB.move(volts);
    RightDB.move(volts);
    pastError = error;
    pros::delay(8);
  }
  LeftDB.brake();
  RightDB.brake();
  pros::delay(500);
}

void pidTurn(double degr){
  double volts, pastError, derivative;
  double kP = 4.20;
  double kD = 0;
  LeftDB.set_zero_position(0);
  RightDB.set_zero_position(0);
  double initialDegr = Gyro.get_heading();
  double error = degr;
  while(fabs(error) > 0.3){
    double reading = Gyro.get_heading() - initialDegr;
    error = degr - reading;
    derivative = (error - pastError);
    volts = (error * kP) - (derivative * kD);
    LeftDB.move(volts);
    RightDB.move(volts);
    pastError = error;
   pros::delay(8);
  }
  LeftDB.brake();
  RightDB.brake();
  pros::delay(500);
}*/


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
	DB.calibrate();
	DB.setPose(0,0,0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
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
	//Auton
	//NOTE: Y is original lateral movement
	//NOTE: X is Perpendicular movement to placement
	//Auton 1
	//DB.moveTo()
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
	//User Control
	while(true){
		pros::delay(20);
		lemlib::Pose pose = DB.getPose(); // get the current position of the robot
        pros::lcd::print(0, "x: %f", pose.x); // print the x position
        pros::lcd::print(1, "y: %f", pose.y); // print the y position
        pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
        pros::delay(10);

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
		if(Master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			Endgame.set_value(true);
		}
	}
}
