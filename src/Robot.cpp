#include "MORESubsystem.h"
#include <queue>
#include "WPILib.h"
#include "AutoStep.h"
#include "DriveTrain.h"

#define FRONT_LEFT_DRIVE_PORT 0
#define REAR_LEFT_DRIVE_PORT 1
#define FRONT_RIGHT_DRIVE_PORT 2
#define REAR_RIGHT_DRIVE_PORT 3

#define LEFT_BUMP_LIMIT_PORT 2
#define RIGHT_BUMP_LIMIT_PORT 1

#define GYRO_PORT 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Joystick* right;
	Joystick* left;
	DriveTrain* drive;
	Gyro* gyro;
	DigitalInput* limit;

	//For autonomous
	queue<AutoStep*> autoSteps;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		right = new Joystick(0);
		left = new Joystick(1);

		gyro = new Gyro(GYRO_PORT);

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, LEFT_BUMP_LIMIT_PORT,
				RIGHT_BUMP_LIMIT_PORT, gyro, "MECANUM");

		limit = new DigitalInput(0);
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
		autoSteps.push(new AutoStep(drive, currentInstr));

		//Second step, drive to the right for 0.5 seconds
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.5;//half speed
		currentInstr.param2 = 90.0;//Go to the right
		currentInstr.param3 = 0;//no rotation
		currentInstr.param4 = 0.5;
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr));

		//Third step, turn for 1 second
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.0;//half speed
		currentInstr.param2 = 0.0;//No rotation
		currentInstr.param3 = -0.5;//half speed rotation
		currentInstr.param4 = 1;
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr));
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
		}
		SmartDashboard::PutNumber("Auto Timer", drive->getAutoTimer());
	}

	void TeleopInit()
	{
		gyro->Reset();
	}

	void TeleopPeriodic()
	{
		drive->Drive(right->GetX(), right->GetY(), right->GetTwist()*-1);
		SmartDashboard::PutNumber("X", right->GetX());
		SmartDashboard::PutNumber("Y", right->GetY()*-1);
		SmartDashboard::PutNumber("Twist", right->GetTwist());
		SmartDashboard::PutNumber("Switch", limit->Get());
		SmartDashboard::PutNumber("Gyro", drive->getGyro()->GetAngle());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
