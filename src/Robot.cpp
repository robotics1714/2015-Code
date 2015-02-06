#include "MORESubsystem.h"
#include <queue>
#include <cmath>
#include "WPILib.h"
#include "AutoStep.h"
#include "AutoTimer.h"
#include "DriveTrain.h"
#include "Spatula.h"
#include "Rake.h"
#include "Lift.h"
#include "XboxController.h"

#define FRONT_LEFT_DRIVE_PORT 0
#define REAR_LEFT_DRIVE_PORT 1
#define FRONT_RIGHT_DRIVE_PORT 2
#define REAR_RIGHT_DRIVE_PORT 3

#define RAKE_DRAW_IN_MOTOR_DEVICE_NUMBER 0
#define RAKE_ACTUATING_SOLENOID_PORT 0
#define RAKE_DRAW_IN_LIMIT_PORT 0

#define LIFT_MOTOR_DEVICE_NUMBER 1
#define LIFT_POT_PORT 2
#define LIFT_ENCO_A_PORT 3
#define LIFT_ENCO_B_PORT 2
#define LIFT_UPPER_BOUND_PORT 4
#define LIFT_LOWER_BOUND_PORT 5

#define SPATULA_MOTOR_DEVICE_NUMBER 2
#define SPATULA_POT_PORT 3

#define LEFT_BUMP_LIMIT_PORT 7
#define RIGHT_BUMP_LIMIT_PORT 6
#define DRIVE_ULTRASONIC_OUT 9
#define DRIVE_ULTRASONIC_IN 8

#define GYRO_PORT 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Joystick* stick;
	XboxController* xbox;
	DriveTrain* drive;
	Spatula* spatula;
	Lift* lift;
	Rake* rake;

	bool spatulaUp;
	bool spatulaMoveButtonPressed;//Used to make sure the spatulaUp variable is toggled only once per button press
	//bool leftTwoButtonPressed;

	//For autonomous
	queue<AutoStep*> autoSteps;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		stick = new Joystick(0);
		xbox = new XboxController(new Joystick(1));

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, LEFT_BUMP_LIMIT_PORT,
				RIGHT_BUMP_LIMIT_PORT, DRIVE_ULTRASONIC_OUT, DRIVE_ULTRASONIC_IN, GYRO_PORT, "MECANUM");

		drive->getGyro()->Reset();

		spatula = new Spatula(SPATULA_MOTOR_DEVICE_NUMBER, SPATULA_POT_PORT, "SPATULA");
		lift = new Lift(LIFT_MOTOR_DEVICE_NUMBER, LIFT_POT_PORT, LIFT_ENCO_A_PORT, LIFT_ENCO_B_PORT,
				LIFT_UPPER_BOUND_PORT, LIFT_LOWER_BOUND_PORT, "LIFT");
		rake = new Rake(RAKE_DRAW_IN_MOTOR_DEVICE_NUMBER, RAKE_ACTUATING_SOLENOID_PORT,
				RAKE_DRAW_IN_LIMIT_PORT, "RAKE");

		spatulaUp = false;
		spatulaMoveButtonPressed = false;

		//leftTwoButtonPressed = false;
	}

	void AutonomousInit()
	{
		drive->getGyro()->Reset();

		AutoInstructions currentInstr;

		//First step, drive backwards towards the step until we are 7.5 inches away
		currentInstr.flags = DriveTrain::ULTRASONIC;
		currentInstr.param1 = -1;//Go quarter speed in reverse
		currentInstr.param2 = 0.0;//Go straight
		currentInstr.param3 = 0.0;//No rotation
		currentInstr.param4 = 7.0;//7 second safety timer
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Don't hit the step"));

		//Second step, wait 0.5 seconds
		currentInstr.flags = 0;
		currentInstr.param1 = 0.5;//Time to wait. the rest of the params are unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(new AutoTimer("Timer"), currentInstr, "Wait"));

		//Third step, drive forwards for 1 second
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.5;//Go half speed
		currentInstr.param2 = 0.0;//Go straight
		currentInstr.param3 = 0.0;//No rotation
		currentInstr.param4 = 0.5;//Go for 1 second
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Drive Back"));
	}

	void AutonomousPeriodic()
	{
		//Check if there are still steps in the queue
		if(!autoSteps.empty())
		{
			//Print out the step of autonomous to smart dashboard
			SmartDashboard::PutString("Auto Step:", autoSteps.front()->GetStepName());
			//Perform the current auto action until it completes
			if(!autoSteps.front()->PerformStep())
			{
				//When the step finishes, remove it from the queue
				autoSteps.pop();
				SmartDashboard::PutString("Done", "Now");
			}
		}
		SmartDashboard::PutNumber("Ultrasonic:", drive->GetUltrasonic()->GetRangeInches());
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
			twist = stick->GetTwist()*(0.6);
		}

		/*//Change between tank and mecanum drive
		if(xbox->Get && !leftTwoButtonPressed)
		{
			mecanum = !mecanum;
			leftTwoButtonPressed = true;
		}
		if(!xbox->GetRawButton(2))
		{
			leftTwoButtonPressed = false;
		}

		if(mecanum)
		{
			drive->Drive(x, y, twist);
		}
		else
		{
			drive->TankDrive(xbox->GetY(), stick->GetY());
		}*/

		//Main driver controls
		if(stick->GetRawButton(3))
		{
			//Aquire a tote/container
			lift->StartAcquire();
		}
		if(stick->GetRawButton(4))
		{
			//Release the stack
		}
		if(stick->GetRawButton(7) && !spatulaMoveButtonPressed)
		{
			//Move the spatula up or down depending on where it is
			if(spatulaUp)
			{
				//Move down
				spatula->StartMoveDown();
			}
			else
			{
				//Move up
				spatula->StartMoveUp();
			}
			//Toggle the spatulaUp variable because it has changed states
			spatulaUp = !spatulaUp;
		}
		spatulaMoveButtonPressed = stick->GetRawButton(7);

		//Second driver controls
		if(xbox->IsAPressed())
		{
			//Go to level 0
			lift->StartMoveToLevel(0);
		}
		else if((xbox->IsXPressed()) && (xbox->GetLeftTriggerAxis() < 0.5))
		{
			//Go to level 1
			lift->StartMoveToLevel(1);
		}
		else if((xbox->IsYPressed()) && (xbox->GetLeftTriggerAxis() < 0.5))
		{
			//Go to level 2
			lift->StartMoveToLevel(2);
		}
		else if((xbox->IsBPressed()) && (xbox->GetLeftTriggerAxis() < 0.5))
		{
			//Go to level 3
			lift->StartMoveToLevel(3);
		}
		else if((xbox->IsXPressed()) && (xbox->GetLeftTriggerAxis() > 0.5))
		{
			//Go to level 4
			lift->StartMoveToLevel(4);
		}
		else if((xbox->IsYPressed()) && (xbox->GetLeftTriggerAxis() > 0.5))
		{
			//Go to level 5
			lift->StartMoveToLevel(5);
		}
		else if((xbox->IsBPressed()) && (xbox->GetLeftTriggerAxis() > 0.5))
		{
			//Go to level 6
			lift->StartMoveToLevel(6);
		}
		if((xbox->GetLeftTriggerAxis() > 0.5) && (fabs(xbox->GetRightYAxis()) > 0.15))
		{
			//Tell the lift to stop if it is doing stuff and then have it move according to the right y axis
			if((lift->GetMovingToLevel()) || (lift->Acquire() != IDLE_STATE))
			{
				lift->Stop();
			}
			lift->Move(xbox->GetRightYAxis());
		}

		drive->Drive(x, y, twist);

		//These functions are for state machines and need to be called every call of the function
		//but will only do something when their respective start function is called
		rake->DrawIn();
		spatula->MoveDown();
		spatula->MoveUp();
		lift->MoveToLevel();//This should only be called once per loop because the integral is time sensitive
		lift->Acquire();

		//Print out information for the driver/debugger
		SmartDashboard::PutNumber("Gyro:", drive->getGyro()->GetAngle());
		SmartDashboard::PutNumber("Ultrasonic:", drive->GetUltrasonic()->GetRangeInches());
		SmartDashboard::PutNumber("Left Limit", drive->GetLeftLimit()->Get());
		SmartDashboard::PutNumber("Right limit", drive->GetRightLimit()->Get());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
