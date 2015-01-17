#include "MORESubsystem.h"
#include <queue>
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
#define SPATULA_MOTOR_DEVICE_NUMBER 2

#define LEFT_BUMP_LIMIT_PORT 2
#define RIGHT_BUMP_LIMIT_PORT 1

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
	DigitalInput* limit;

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

		limit = new DigitalInput(0);

		spatula = new Spatula(SPATULA_MOTOR_DEVICE_NUMBER, "SPATULA");
		lift = new Lift(LIFT_MOTOR_DEVICE_NUMBER, "LIFT");
		rake = new Rake(RAKE_WINCH_DEVICE_NUMBER, "RAKE");
	}

	void AutonomousInit()
	{
		AutoInstructions currentInstr;

		//First step, drive forward for 0.5 seconds
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.5;//Go half speed
		currentInstr.param2 = 0.0;//Go straight
		currentInstr.param3 = 0.0;//No rotation
		currentInstr.param4 = 0.5;
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Go Forward"));

		//Second step, drive to the right for 0.5 seconds
		currentInstr.flags = DriveTrain::TIME;
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
		autoSteps.push(new AutoStep(drive, currentInstr, "Turn"));
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
		SmartDashboard::PutNumber("Auto Timer", drive->getAutoTimer());
	}

	void TeleopInit()
	{
		gyro->Reset();
	}

	void TeleopPeriodic()
	{
		drive->Drive(stick->GetX(), stick->GetY(), stick->GetTwist()*-1);
		SmartDashboard::PutNumber("X", stick->GetX());
		SmartDashboard::PutNumber("Y", stick->GetY()*-1);
		SmartDashboard::PutNumber("Twist", stick->GetTwist());
		SmartDashboard::PutNumber("Switch", limit->Get());
		SmartDashboard::PutNumber("Gyro", drive->getGyro()->GetAngle());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
