#include "MORESubsystem.h"
#include <queue>
#include <cmath>
#include "WPILib.h"
#include "AutoStep.h"
#include "DriveTrain.h"
#include "Spatula.h"
#include "Rake.h"
#include "Lift.h"

#define FRONT_LEFT_DRIVE_PORT 0
#define REAR_LEFT_DRIVE_PORT 1
#define FRONT_RIGHT_DRIVE_PORT 2
#define REAR_RIGHT_DRIVE_PORT 3

#define RAKE_WINCH_DEVICE_NUMBER 0

#define LIFT_MOTOR_DEVICE_NUMBER 1
#define LIFT_ENCO_A_PORT 2
#define LIFT_ENCO_B_PORT 3
#define LIFT_UPPER_BOUND_PORT 4
#define LIFT_LOWER_BOUND_PORT 5

#define SPATULA_MOTOR_DEVICE_NUMBER 2
#define SPATULA_POT_PORT 1

#define LEFT_BUMP_LIMIT_PORT 1
#define RIGHT_BUMP_LIMIT_PORT 0

#define GYRO_PORT 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Joystick* stick;
	DriveTrain* drive;
	Spatula* spatula;
	Lift* lift;
	Rake* rake;
	Gyro* gyro;

	//For autonomous
	queue<AutoStep*> autoSteps;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		stick = new Joystick(0);

		gyro = new Gyro(GYRO_PORT);

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, LEFT_BUMP_LIMIT_PORT,
				RIGHT_BUMP_LIMIT_PORT, gyro, "MECANUM");

		drive->getGyro()->Reset();

		spatula = new Spatula(SPATULA_MOTOR_DEVICE_NUMBER, SPATULA_POT_PORT, "SPATULA");
		lift = new Lift(LIFT_MOTOR_DEVICE_NUMBER, LIFT_ENCO_A_PORT, LIFT_ENCO_B_PORT,
				LIFT_UPPER_BOUND_PORT, LIFT_LOWER_BOUND_PORT, "LIFT");
		rake = new Rake(RAKE_WINCH_DEVICE_NUMBER, "RAKE");
	}

	void AutonomousInit()
	{
		drive->getGyro()->Reset();

		AutoInstructions currentInstr;

		//First step, drive forward for 7 seconds
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.75;//Go half speed
		currentInstr.param2 = 0.0;//Go straight
		currentInstr.param3 = 0.0;//No rotation
		currentInstr.param4 = 4.5;//turn for 7 seconds
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Go Forward"));

		//Second step, drive to the right for 0.5 seconds
		/*currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.5;//half speed
		currentInstr.param2 = 90.0;//Go to the right
		currentInstr.param3 = 0;//no rotation
		currentInstr.param4 = 0.5;
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Strafe Right"));

		//Third step, turn for 1 second
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.0;//half speed
		currentInstr.param2 = 0.0;//No rotation
		currentInstr.param3 = -0.5;//half speed rotation
		currentInstr.param4 = 1;
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Turn"));*/
	}

	void AutonomousPeriodic()
	{
		//Check if there are still steps in the queue
		if(!autoSteps.empty())
		{
			//Perform the current auto action until it completes
			if(!autoSteps.front()->PerformStep())
			{
				//When the step finishes, remove it from the queue
				autoSteps.pop();
			}
			//Print out the step of autonomous to smart dashboard
			SmartDashboard::PutString("Auto Step:", autoSteps.front()->GetStepName());
		}
		/*if(stick->GetRawButton(1))
		{
			AutoInstructions currentInstr;

			//First step, drive forward for 7 seconds
			currentInstr.flags = DriveTrain::TIME;
			currentInstr.param1 = 0.55;//Go half speed
			currentInstr.param2 = 0.0;//Go straight
			currentInstr.param3 = 0.0;//No rotation
			currentInstr.param4 = 7;//turn for 7 seconds
			//Add the step to the queue
			autoSteps.push(new AutoStep(drive, currentInstr, "Go Forward"));
		}*/
		SmartDashboard::PutNumber("Auto Timer", drive->getAutoTimer());
		SmartDashboard::PutNumber("Gyro", drive->getGyro()->GetAngle());
	}

	void TeleopInit()
	{
		//drive->getGyro()->Reset();
	}

	void TeleopPeriodic()
	{
		float x, y, twist;
		x = y = twist = 0.0;

		//Filter out values coming from the joystick not returning to center
		if(fabs(stick->GetX()) > 0.12)
		{
			x = stick->GetX();
		}
		if(fabs(stick->GetY()) > 0.12)
		{
			y = stick->GetY();
		}
		if((fabs(stick->GetTwist()) > 0.12) && (!stick->GetRawButton(1)))
		{
			twist = stick->GetTwist()*(-0.6);
		}

		drive->Drive(x, y, twist);
		SmartDashboard::PutNumber("X", stick->GetX());
		SmartDashboard::PutNumber("Y", stick->GetY()*-1);
		SmartDashboard::PutNumber("Twist", stick->GetTwist());
		SmartDashboard::PutNumber("Gyro", drive->getGyro()->GetAngle());
		SmartDashboard::PutNumber("Current Heading", drive->GetCurrentHeading());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
