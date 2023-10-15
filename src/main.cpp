// Include classess and API
#include "main.h"
#include "api.h"
//#include "classess.cpp"

//Temperary declarations

//Declare Controller
pros::Controller Master(pros::E_CONTROLLER_MASTER);

//Declare Motors

pros::Motor MotorLeftF(11, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MotorLeftB(17, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MotorRightF(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor MotorRightB(7, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor Intake(20, pros::E_MOTOR_GEARSET_18, false, pros::
E_MOTOR_ENCODER_DEGREES);
pros::Motor Cata(10, pros::E_MOTOR_GEARSET_36, true, pros::E_MOTOR_ENCODER_DEGREES);

//Declare Motor Groups
pros::Motor_Group LeftDB({MotorLeftF, MotorLeftB});
pros::Motor_Group RightDB({MotorRightF, MotorRightB});


//Methods for Auton and Driver control

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


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
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
	//Initilization (Interial sensor)
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

		//Declare variables for arcade drive
		double power = Master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double turn = Master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		//Declare Arcade drive
		RightDB.move(power - turn);
		LeftDB.move(power + turn);

		//Run intake
		if(Master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake(127);
		}
        else if(Master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){ //Outtake
			intake(-127);
		}
        else{
            intake(0);
        }
		
		//Run Cata
		if(Master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			cataLaunch(127);
		}
		else{
			cataLaunch(0);
		}
	}
	}
