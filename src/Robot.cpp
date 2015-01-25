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
#define RAKE_LIMIT_PORT 0

#define LIFT_MOTOR_DEVICE_NUMBER 1
#define LIFT_POT_PORT 2
#define LIFT_ENCO_A_PORT 3
#define LIFT_ENCO_B_PORT 2
#define LIFT_UPPER_BOUND_PORT 4
#define LIFT_LOWER_BOUND_PORT 5

#define SPATULA_MOTOR_DEVICE_NUMBER 2
#define SPATULA_POT_PORT 3

#define LEFT_BUMP_LIMIT_PORT 1
#define RIGHT_BUMP_LIMIT_PORT 0
#define DRIVE_ULTRASONIC_OUT 8
#define DRIVE_ULTRASONIC_IN 9

#define GYRO_PORT 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Joystick* rightStick;
	Joystick* leftStick;
	DriveTrain* drive;
	Spatula* spatula;
	Lift* lift;
	Rake* rake;
	AnalogInput* portOne;

	//Keeps track if the robot has mecanum drive
	bool mecanum;
	bool leftTwoButtonPressed;

	//For autonomous
	queue<AutoStep*> autoSteps;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		portOne = new AnalogInput(1);

		rightStick = new Joystick(0);
		leftStick = new Joystick(1);

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, LEFT_BUMP_LIMIT_PORT,
				RIGHT_BUMP_LIMIT_PORT, DRIVE_ULTRASONIC_OUT, DRIVE_ULTRASONIC_IN, GYRO_PORT, "MECANUM");

		drive->getGyro()->Reset();

		spatula = new Spatula(SPATULA_MOTOR_DEVICE_NUMBER, SPATULA_POT_PORT, "SPATULA");
		lift = new Lift(LIFT_MOTOR_DEVICE_NUMBER, LIFT_POT_PORT, LIFT_ENCO_A_PORT, LIFT_ENCO_B_PORT,
				LIFT_UPPER_BOUND_PORT, LIFT_LOWER_BOUND_PORT, "LIFT");
		rake = new Rake(RAKE_WINCH_DEVICE_NUMBER, RAKE_LIMIT_PORT, "RAKE");

		mecanum = true;
		leftTwoButtonPressed = false;
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
		if(fabs(rightStick->GetX()) > 0.12)
		{
			x = rightStick->GetX();
		}
		if(fabs(rightStick->GetY()) > 0.12)
		{
			y = rightStick->GetY();
		}
		if((fabs(rightStick->GetTwist()) > 0.12) && (!rightStick->GetRawButton(1)))
		{
			twist = rightStick->GetTwist()*(0.6);
		}

		//Change between tank and mecanum drive
		if(leftStick->GetRawButton(2) && !leftTwoButtonPressed)
		{
			mecanum = !mecanum;
			leftTwoButtonPressed = true;
		}
		if(!leftStick->GetRawButton(2))
		{
			leftTwoButtonPressed = false;
		}

		if(mecanum)
		{
			drive->Drive(x, y, twist);
		}
		else
		{
			drive->TankDrive(leftStick->GetY(), rightStick->GetY());
		}

		//These functions are for state machines and need to be called every call of the function
		//but will only do something when their respective start function is called
		rake->MoveForTime();
		spatula->MoveDown();
		spatula->MoveUp();
		lift->MoveToLevel();
		lift->Acquire();

		//Print out information for the driver/debugger
		SmartDashboard::PutNumber("X", rightStick->GetX());
		SmartDashboard::PutNumber("Y", rightStick->GetY()*-1);
		SmartDashboard::PutNumber("Twist", rightStick->GetTwist());
		SmartDashboard::PutNumber("Distance: ", spatula->GetPot()->GetAverageValue());
		SmartDashboard::PutNumber("Gyro:", drive->getGyro()->GetAngle());
		SmartDashboard::PutNumber("Rate: ", lift->GetEnco()->GetRate());
		SmartDashboard::PutNumber("AI 2:", lift->GetPot()->GetAverageValue());
		SmartDashboard::PutNumber("AI 1:", portOne->GetAverageValue());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
