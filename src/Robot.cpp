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
#define RAKE_UP_ACTUATING_SOLENOID_PORT 1
#define RAKE_DOWN_ACTUATING_SOLENOID_PORT 0
#define RAKE_DRAW_IN_LIMIT_PORT 3

#define LIFT_MOTOR_DEVICE_NUMBER 1
#define LIFT_POT_PORT 1
#define LIFT_ENCO_A_PORT 1
#define LIFT_ENCO_B_PORT 2
#define LIFT_UPPER_BOUND_PORT 5
#define LIFT_LOWER_BOUND_PORT 4

#define SPATULA_MOTOR_DEVICE_NUMBER 2
#define SPATULA_POT_PORT 2

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
	DriveTrain* drive;
	Spatula* spatula;
	Lift* lift;
	Rake* rake;

	bool spatulaUp;
	bool spatulaMoveButtonPressed;//Used to make sure the spatulaUp variable is toggled only once per button press
	bool rakeUp;
	bool rakeMoveButtonPressed;
	//bool leftTwoButtonPressed;

	//For autonomous
	queue<AutoStep*> autoSteps;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		stick = new Joystick(0);

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, LEFT_BUMP_LIMIT_PORT,
				RIGHT_BUMP_LIMIT_PORT, DRIVE_ULTRASONIC_OUT, DRIVE_ULTRASONIC_IN, GYRO_PORT, "MECANUM");

		drive->getGyro()->Reset();

		spatula = new Spatula(SPATULA_MOTOR_DEVICE_NUMBER, SPATULA_POT_PORT, "SPATULA");
		lift = new Lift(LIFT_MOTOR_DEVICE_NUMBER, LIFT_POT_PORT, LIFT_ENCO_A_PORT, LIFT_ENCO_B_PORT,
				LIFT_UPPER_BOUND_PORT, LIFT_LOWER_BOUND_PORT, spatula, "LIFT");
		rake = new Rake(RAKE_DRAW_IN_MOTOR_DEVICE_NUMBER, RAKE_UP_ACTUATING_SOLENOID_PORT, RAKE_DOWN_ACTUATING_SOLENOID_PORT,
				RAKE_DRAW_IN_LIMIT_PORT, "RAKE");

		spatulaUp = false;
		spatulaMoveButtonPressed = false;
		rakeUp = false;
		rakeMoveButtonPressed = false;

		SmartDashboard::PutNumber("Auto Delay", 0.0);

		//leftTwoButtonPressed = false;

		//Move the rake up
		rake->MoveUp();
	}

	void AutonomousInit()
	{
		drive->getGyro()->Reset();

		//Get how long to wait at the beginning of autonomous from SmartDashboard
		double wait = SmartDashboard::GetNumber("Auto Delay");

		AutoInstructions currentInstr;

		//First step, wait for however long the drivers decided before the match
		currentInstr.flags = 0;//AutoTimer does not have any flags
		currentInstr.param1 = wait;//How long to wait
		currentInstr.param2 = 0;//unused
		currentInstr.param3 = 0;//unused
		currentInstr.param4 = 0;//unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(new AutoTimer("Timer"), currentInstr, "First Wait"));

		//Second step, drive backwards towards the step until we are 7.5 inches away
		currentInstr.flags = DriveTrain::ULTRASONIC;
		currentInstr.param1 = -1.0;//Go quarter speed in reverse
		currentInstr.param2 = 0.0;//Go straight
		currentInstr.param3 = 0.0;//No rotation
		currentInstr.param4 = 7.0;//7 second safety timer
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Don't hit the step"));

		//Wait a bit
		currentInstr.flags = 0;
		currentInstr.param1 = 1;//Wait
		currentInstr.param2 = 0;//unused
		currentInstr.param3 = 0;//unused
		currentInstr.param4 = 0;//unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(new AutoTimer("Timer"), currentInstr, "Wait"));

		//Drop the rake
		currentInstr.flags = Rake::AUTO_MOVE_DOWN;
		currentInstr.param1 = currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0.0;//Unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(rake, currentInstr, "Drop Rake"));

		//Third step, wait 0.5 seconds and drop the intake
		currentInstr.flags = 0;
		currentInstr.param1 = 1;//Time to wait. the rest of the params are unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(new AutoTimer("Timer"), currentInstr, "Wait"));

		//Fourth step, drive forwards for 1 second
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 1.0;//Go half speed
		currentInstr.param2 = 0.0;//Go straight
		currentInstr.param3 = 0.0;//No rotation
		currentInstr.param4 = 0.5;//Go for 1 second
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Drive Back"));

		//Bring the rake up
		currentInstr.flags = Rake::AUTO_MOVE_UP;
		currentInstr.param1 = currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0.0;//Unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(rake, currentInstr, "Raise Rake"));

		//Bring in the extensions
		/*currentInstr.flags = Rake::AUTO_DRAW_IN;
		currentInstr.param1 = -1;//Speed to draw in
		currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0;
		//Add the step to the queue
		autoSteps.push(new AutoStep(rake, currentInstr, "Draw in extensions"));*/
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
		//Move the spatula
		if(stick->GetRawButton(5) && !spatulaMoveButtonPressed)
		{
			//If the spatula is up, move down
			if(spatulaUp)
			{
				spatula->StartMoveDown();
			}
			//If the spatula is down, move it up
			else
			{
				spatula->StartMoveUp();
			}
			//Toggle the spatula
			spatulaUp = !spatulaUp;
		}
		spatulaMoveButtonPressed = stick->GetRawButton(5);

		//Actuate the take
		if(stick->GetRawButton(2) && !rakeMoveButtonPressed)
		{
			//If the rake is up, move it down
			if(rakeUp)
			{
				rake->MoveDown();
			}
			//If the rake is down, move it up
			else
			{
				rake->MoveUp();
			}
			//Toggle spatula down
			rakeUp = !rakeUp;
		}
		rakeMoveButtonPressed = stick->GetRawButton(2);

		/*else if(stick->GetRawButton(3))
		{
			spatula->StartMoveDown();
		}
		else if((!spatula->MoveUp()) && (!spatula->MoveDown()))
		{
			spatula->Rotate(0);
		}*/

		//Raise the lift up/down
		if(stick->GetRawButton(6))
		{
			//Up
			lift->Move(-1.0);
			lift->SetManuallyMoving(true);
		}
		else if(stick->GetRawButton(4))
		{
			//Down
			lift->Move(0.75);
			lift->SetManuallyMoving(true);
		}
		else
		{
			lift->Move(0);
		}


		//Controls to move the lift to each level
		if(stick->GetRawButton(12))
		{
			lift->StartMoveToLevel(0);
		}
		else if(stick->GetRawButton(11))
		{
			lift->StartMoveToLevel(1);
		}
		else if(stick->GetRawButton(10))
		{
			lift->StartMoveToLevel(2);
		}
		else if(stick->GetRawButton(9))
		{
			lift->StartMoveToLevel(3);
		}
		else if(stick->GetRawButton(8))
		{
			lift->StartMoveToLevel(4);
		}
		else if(stick->GetRawButton(7))
		{
			lift->StartMoveToLevel(5);
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
		SmartDashboard::PutNumber("Lift Upper Bound", lift->getUpperBound()->Get());
		SmartDashboard::PutNumber("Lift Lower Bound", lift->getLowerBound()->Get());
		SmartDashboard::PutNumber("Lift Velocity", lift->GetEnco()->GetRate());
		SmartDashboard::PutNumber("Lift Enco Pos", lift->GetEnco()->GetDistance());
		SmartDashboard::PutNumber("Lift Enco Pulses", lift->GetEnco()->GetRaw());
		SmartDashboard::PutNumber("Lift Position", lift->GetPot()->GetAverageValue());
		SmartDashboard::PutNumber("Spat Position", spatula->GetPot()->GetAverageValue());
		SmartDashboard::PutNumber("Gyro", drive->getGyro()->GetAngle());
	}

	void DisabledPeriodic()
	{
		drive->ResetAutoCorrect();
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
