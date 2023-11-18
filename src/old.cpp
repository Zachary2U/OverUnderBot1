/*

This entire file is just to store older Autons for future reference or use

*/
/*
if(right){
		//Auton RightSide
		//Move forward, Turn right, Put matchload into goal
		DB.moveTo(0, 41, 1000);
		DB.turnTo(30, 41, 1000);
		intake(-90);
		pros::delay(1000);
		MotorGroupDriveBase.move(127);
		pros::delay(2000);
		intake(0);
		MotorGroupDriveBase.brake();
	}
	else if(left){
		//Auton LeftSide
		//Move forward, Turn right, Put matchload into goal
		DB.moveTo(0, 41, 1000);
		DB.turnTo(-30, 41, 1000);
		intake(-90);
		pros::delay(1000);
		MotorGroupDriveBase.move(127);
		pros::delay(2000);
		intake(0);
		MotorGroupDriveBase.brake();
	}
	else{
		cataLaunch(127);
		pros::delay(2000);
		cataLaunch(0);
	}
    */

	/* Ridgefield Right Side
	// Move bot off of starting position
		MotorGroupDriveBase.move(127);
		pros::delay(650);
		MotorGroupDriveBase.move(63);
		pros::delay(280);
		
		//Turn to face the goal
		DB.turnTo(1000, DB.getPose().y, 1000);
		MotorGroupDriveBase.brake();
		pros::delay(500);

		//Outtake triball
		intake(-70);
		pros::delay(500);
		
		//Shift back as to not affect the triball to be scored
		MotorGroupDriveBase.move(-20);
		pros::delay(100);
		MotorGroupDriveBase.brake();

		//Turn bot to hit with rear
		DB.turnTo(-1000, DB.getPose().y, 1000);

		//Hit triball into goal
		MotorGroupDriveBase.move(-127);
		pros::delay(750);
		MotorGroupDriveBase.move(-70); //Make sure robot is against the goal for next step
		pros::delay(500);
		MotorGroupDriveBase.brake();

		//Turn to get the center-most triball
		DB.turnTo(-200, 220, 1000);
		intake(127);

		//Move to get triball
		MotorGroupDriveBase.move(127);
		pros::delay(300);
		MotorGroupDriveBase.brake();

		//Turn around and deposit triball in front of the goal
		DB.turnTo(1000, DB.getPose().y, 1000);
		pros::delay(400);
		intake(-70);

		//Turn to face final triball and intake
		MotorGroupDriveBase.move(-40);
		pros::delay(150);
		DB.turnTo(-1000, DB.getPose().y, 1000);
		intake(127);
		MotorGroupDriveBase.move(80);
		pros::delay(750);
		MotorGroupDriveBase.brake();
		
		//Move back
		MotorGroupDriveBase.move(-50);
		pros::delay(550);
		MotorGroupDriveBase.brake();
		DB.turnTo(1000, DB.getPose().y, 1000);
		pros::delay(1000);
		intake(-70);
		pros::delay(1000);
		intake(0);

		//Back up and turn around
		MotorGroupDriveBase.move(-50);
		pros::delay(400);
		MotorGroupDriveBase.brake();
		DB.turnTo(-1000, DB.getPose().y, 1000);

		MotorGroupDriveBase.move(-127);
		pros::delay(2000);
		MotorGroupDriveBase.brake();*/

		/*// Move bot off of starting position
		MotorGroupDriveBase.move(127);
		pros::delay(650);
		MotorGroupDriveBase.move(63);
		pros::delay(280);
		
		//Turn to face the goal
		DB.turnTo(-1000, DB.getPose().y, 1000);
		MotorGroupDriveBase.brake();
		pros::delay(500);

		//Outtake triball
		intake(-70);
		pros::delay(500);
		
		//Shift back as to not affect the triball to be scored
		MotorGroupDriveBase.move(-20);
		pros::delay(100);
		MotorGroupDriveBase.brake();

		//Turn bot to hit with rear
		DB.turnTo(1000, DB.getPose().y, 1000);

		//Hit triball into goal
		MotorGroupDriveBase.move(-127);
		pros::delay(750);
		MotorGroupDriveBase.move(-70); //Make sure robot is against the goal for next step
		pros::delay(500);
		MotorGroupDriveBase.brake();
		MotorGroupDriveBase.move(127);
		pros::delay(500);
		MotorGroupDriveBase.brake();*/