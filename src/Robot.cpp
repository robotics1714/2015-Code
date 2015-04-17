#include "MORESubsystem.h"
#include <queue>
#include <cmath>
#include <fstream>
#include <iostream>
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
#define LIFT_POT_PORT 2
//#define LIFT_ENCO_A_PORT 1
//#define LIFT_ENCO_B_PORT 2
#define LIFT_UPPER_BOUND_PORT 5
#define LIFT_LOWER_BOUND_PORT 4

#define SPATULA_MOTOR_DEVICE_NUMBER 2
#define SPATULA_ENCO_A_PORT 1
#define SPATULA_ENCO_B_PORT 2
#define SPATULA_OPEN_LIMIT_PORT 0
//#define SPATULA_POT_PORT 2

#define LEFT_BUMP_LIMIT_PORT 7
#define RIGHT_BUMP_LIMIT_PORT 6
#define DRIVE_ULTRASONIC_OUT 9
#define DRIVE_ULTRASONIC_IN 8

#define YAW_GYRO_PORT 0
#define PITCH_GYRO_PORT 1

//Defines for the keys for the auto choices
#define AUTO_DRIVE "DRIVE"
#define AUTO_RELEASE "RELEASE"
#define AUTO_KEEP "KEEP"

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
	bool rakeUp;
	bool rakeMoveButtonPressed;
	bool moveToLevelButtonPressed;
	//bool leftTwoButtonPressed;

	//For autonomous
	queue<AutoStep*> autoSteps;
	SendableChooser* autoChooser;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		stick = new Joystick(0);
		xbox = new XboxController(new Joystick(1));

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, LEFT_BUMP_LIMIT_PORT,
				RIGHT_BUMP_LIMIT_PORT, DRIVE_ULTRASONIC_OUT, DRIVE_ULTRASONIC_IN, YAW_GYRO_PORT, PITCH_GYRO_PORT, "MECANUM");

		drive->getYawGyro()->Reset();

		//Read in whether this is robot 1 or 2
		string robotNumber;
		ifstream file;
		file.open("/home/lvuser/robot.txt");
		//Check if the file opened correctly
		if(!file.is_open())
		{
			//If not, set it to the competition robot by default
			robotNumber = "1";
		}
		else
		{
			cout<<"Opened"<<endl;
			//If it opened correctly, read the file
			file>>robotNumber;
		}
		file.close();

		spatula = new Spatula(SPATULA_MOTOR_DEVICE_NUMBER, SPATULA_ENCO_A_PORT, SPATULA_ENCO_B_PORT,
				SPATULA_OPEN_LIMIT_PORT, robotNumber, "SPATULA");
		lift = new Lift(LIFT_MOTOR_DEVICE_NUMBER, LIFT_POT_PORT, LIFT_UPPER_BOUND_PORT, LIFT_LOWER_BOUND_PORT,
				spatula, robotNumber, "LIFT");
		rake = new Rake(RAKE_DRAW_IN_MOTOR_DEVICE_NUMBER, RAKE_UP_ACTUATING_SOLENOID_PORT, RAKE_DOWN_ACTUATING_SOLENOID_PORT,
				RAKE_DRAW_IN_LIMIT_PORT, "RAKE");

		spatulaUp = false;
		spatulaMoveButtonPressed = false;
		rakeUp = false;
		rakeMoveButtonPressed = false;
		moveToLevelButtonPressed = false;

		SmartDashboard::PutNumber("Auto Delay", 0.0);

		//leftTwoButtonPressed = false;

		//Move the rake down
		rake->MoveDown();

		//Set up the choices for auto
		autoChooser = new SendableChooser();
		autoChooser->AddDefault("Keep Containers", new string(AUTO_KEEP));
		//autoChooser->AddObject("Re-Rake", new string("RE"));
		autoChooser->AddObject("Release Containers", new string(AUTO_RELEASE));
		autoChooser->AddObject("Just Drive", new string(AUTO_DRIVE));
		//Put the chooser on SmartDashboard
		SmartDashboard::PutData("Auto Selection", autoChooser);


		//Print which robot we're using
		SmartDashboard::PutString("Robot: ", (robotNumber == "2")?"Practice Bot":"Competition Bot");
		cout<<robotNumber;
	}

	void AutonomousInit()
	{
		drive->getYawGyro()->Reset();
		string autoChoice = *(string*)autoChooser->GetSelected();

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

		//lower the rake
		/*if(autoChoice != "DRIVE")
		{
			currentInstr.flags = Rake::AUTO_MOVE_DOWN;
			currentInstr.param1 = currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0.0;//Unused
			//Add the step to the queue
			autoSteps.push(new AutoStep(rake, currentInstr, "Drop Rake"));
		}*/

		//Second step, drive backwards towards the step until we are10 inches away
		currentInstr.flags = DriveTrain::ULTRASONIC | DriveTrain::ULTRASONIC_5IN;
		currentInstr.param1 = -0.8;//Go quarter speed in reverse
		currentInstr.param2 = 0.0;//Go straight
		currentInstr.param3 = 0.0;//No rotation
		currentInstr.param4 = 7.0;//7 second safety timer
		autoSteps.push(new AutoStep(drive, currentInstr, "Don't hit the step"));

		//Wait a bit and have the robot center itself
		currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0;
		currentInstr.param2 = 0;
		currentInstr.param3 = 0;
		currentInstr.param4 = 0.45;
		autoSteps.push(new AutoStep(drive, currentInstr, "wait and fix itself"));
		/*currentInstr.flags = 0;
		currentInstr.param1 = 1;//Wait
		currentInstr.param2 = 0;//unused
		currentInstr.param3 = 0;//unused
		currentInstr.param4 = 0;//unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(new AutoTimer("Timer"), currentInstr, "Wait"));*/

		//Drop the rake
		/*currentInstr.flags = Rake::AUTO_MOVE_DOWN;
		currentInstr.param1 = currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0.0;//Unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(rake, currentInstr, "Drop Rake"));

		//Third step, wait 0.5 seconds and drop the intake
		currentInstr.flags = 0;
		currentInstr.param1 = 1.5;//Time to wait. the rest of the params are unused
		//Add the step to the queue
		autoSteps.push(new AutoStep(new AutoTimer("Timer"), currentInstr, "Wait"));*/

		//THIS IS THE DRIVE AWAY SEQUENCE THAT HAS A BURST OF SPEED AND THEN RAMPS UP THE SPEED
		//Have a short burst of speed to dislodge the cantainers
		/*currentInstr.flags = DriveTrain::TIME;
		currentInstr.param1 = 0.5;//Speed
		currentInstr.param2 = 0;//Direction
		currentInstr.param3 = 0;//Rotation
		currentInstr.param4 = 0.75;//Time
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Short Burst"));

		//Ramp up
		currentInstr.flags = DriveTrain::RAMP_UP;
		currentInstr.param1 = 0.75;//Speed
		currentInstr.param2 = 0;//Direction
		currentInstr.param3 = 0;//Rotation
		currentInstr.param4 = 1.5;//Time
		//Add the step to the queue
		autoSteps.push(new AutoStep(drive, currentInstr, "Ramp Up"));*/

		//THIS IS THE KNOWN DRIVE AWAY STEP THAT DOES NOT (COMPLETELY) TIP OVER THE ROBOT
		//Choose how long we go for driving back
		if(autoChoice != AUTO_DRIVE)
		{
			//Raise the rake
			currentInstr.flags = Rake::AUTO_MOVE_UP;
			currentInstr.param1 = currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0.0;//Unused
			//Add the step to the queue
			autoSteps.push(new AutoStep(rake, currentInstr, "Aquire Containers"));

			//Delay to grab the cans
			currentInstr.flags = DriveTrain::TIME;
			currentInstr.param1 = 0;
			currentInstr.param2 = 0;
			currentInstr.param3 = 0;
			currentInstr.param4 = 1;
			autoSteps.push(new AutoStep(drive, currentInstr, "wait and grab containers"));

			float driveBackTime;
			float driveBackSpeed;
			//Auto where we release the containers
			if(autoChoice == AUTO_RELEASE)
			{
				driveBackTime = 4.5;
				driveBackSpeed = 0.35;
			}
			//Auto where we hold onto the containers
			else
			{
				driveBackTime = 4.5;
				driveBackSpeed = 0.35;
			}
			//Fourth step, drive forwards away from the step
			currentInstr.flags = DriveTrain::TIME;
			currentInstr.param1 = driveBackSpeed;//Go half speed
			currentInstr.param2 = 0.0;//Go straight
			currentInstr.param3 = 0.0;//No rotation
			currentInstr.param4 = driveBackTime;//Time
			//Add the step to the queue
			autoSteps.push(new AutoStep(drive, currentInstr, "Drive Back Ultra"));

			//Lower the rake
			currentInstr.flags = Rake::AUTO_MOVE_DOWN;
			currentInstr.param1 = currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0.0;//Unused
			//Add the step to the queue
			autoSteps.push(new AutoStep(rake, currentInstr, "Release Containers"));

			if(autoChoice == AUTO_RELEASE)
			{
				//Wait before bringing in the extensions
				currentInstr.flags = 0;
				currentInstr.param1 = 0.3;
				currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0;
				//Add the step to the queue
				autoSteps.push(new AutoStep(new AutoTimer("Timer"), currentInstr, "Wait"));

				//Drive back to release the containers
				currentInstr.flags = DriveTrain::TIME;
				currentInstr.param1 = -0.25;//Speed
				currentInstr.param2 = 0;//Direction
				currentInstr.param3 = 0;//Rotation
				currentInstr.param4 = 1.5;//Time
				autoSteps.push(new AutoStep(drive, currentInstr, "Release Cans"));

				//Bring in the extensions
				currentInstr.flags = Rake::AUTO_DRAW_IN;
				currentInstr.param1 = -1;//Speed to draw in
				currentInstr.param2 = currentInstr.param3 = currentInstr.param4 = 0;
				//Add the step to the queue
				autoSteps.push(new AutoStep(rake, currentInstr, "Draw in extensions"));
			}

			//Straighten out the robot
			currentInstr.flags = DriveTrain::TIME;
			currentInstr.param1 = 0;//Speed
			currentInstr.param2 = 0;//Direction
			currentInstr.param3 = 0;//Rotation
			currentInstr.param4 = 0.75;//Time
			//Add the step to the queue
			autoSteps.push(new AutoStep(drive, currentInstr, "Straighten robot"));
		}
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
		drive->UpdateAdjustmentVal();

		SmartDashboard::PutNumber("Ultrasonic:", drive->GetUltrasonic()->GetRangeInches());
		SmartDashboard::PutNumber("Yaw Gyro", drive->getYawGyro()->GetAngle());
	}

	void TeleopInit()
	{
		//drive->getGyro()->Reset();
		drive->getPitchGyro()->Reset();
		drive->ResetAntiTip();
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
		if((fabs(stick->GetTwist()) > 0.3) && (!stick->GetRawButton(1)))
		{
			twist = stick->GetTwist()*(0.35);
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
			//If the spatula is moving it, stop that process
			spatula->Stop();
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

		//Actuate the rake
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
			//lift->override(-0.5);
			lift->SetManuallyMoving(true);
		}
		else if(stick->GetRawButton(4))
		{
			//Down
			lift->Move(0.75);
			//lift->override(0.5);
			lift->SetManuallyMoving(true);
		}
		else if(!lift->GetMovingToLevel())
		{
			lift->Move(0);
		}


		//Controls to move the lift to each level
		if(stick->GetRawButton(12) && !moveToLevelButtonPressed)
		{
			lift->Stop();
			lift->StartMoveToLevel(0);
		}
		else if(stick->GetRawButton(11) && !moveToLevelButtonPressed)
		{
			lift->Stop();
			lift->StartMoveToLevel(1);
		}
		else if(stick->GetRawButton(10) && !moveToLevelButtonPressed)
		{
			lift->Stop();
			lift->StartMoveToLevel(2);
		}
		else if(stick->GetRawButton(9) && !moveToLevelButtonPressed)
		{
			lift->Stop();
			lift->StartMoveToLevel(3);
		}
		else if(stick->GetRawButton(8) && !moveToLevelButtonPressed)
		{
			lift->Stop();
			lift->StartMoveToLevel(4);
		}
		else if(stick->GetRawButton(7) && !moveToLevelButtonPressed)
		{
			lift->Stop();
			lift->StartMoveToLevel(5);
		}
		//Used to make sure telling the lift to move to a level happens once per button press
		moveToLevelButtonPressed = (stick->GetRawButton(12) || stick->GetRawButton(11) ||
				stick->GetRawButton(10) || stick->GetRawButton(9) || stick->GetRawButton(8) || stick->GetRawButton(7));


		//Bring in the extensions
		if(stick->GetRawButton(3))
		{
			rake->StartDrawIn(-1);
		}
		//Roll out the spool
		if(xbox->IsAPressed())
		{
			rake->Move(0.25);
		}
		else if(xbox->GetRightTriggerAxis() > 0.5)
		{
			rake->Move(-1);
		}
		else if(!rake->DrawIn())
		{
			rake->Move(0);
		}

		//manual spatula commands
		if(xbox->IsXPressed())
		{
			spatula->Rotate(0.25);
		}
		else if(xbox->IsBPressed())
		{
			spatula->Rotate(-0.25);
		}
		else if(!spatula->MoveDown() && !spatula->MoveUp())
		{
			spatula->Rotate(0);
		}

		//Button for resseting the gyro just incase
		if(xbox->IsLeftBumperPressed() && xbox->IsRightBumperPressed())
		{
			drive->ResetAutoCorrect();
		}
		if(xbox->IsYPressed())
		{
			drive->ResetAntiTip();
		}

		drive->Drive(x, y, twist);

		//These functions are for state machines and need to be called every call of the function
		//but will only do something when their respective start function is called
		rake->DrawIn();
		spatula->MoveDown();
		spatula->MoveUp();
		lift->MoveToLevel();//This should only be called once per loop because the integral is time sensitive
		lift->Acquire();

		//drive->CorrectPitchGyro();
		drive->UpdateAdjustmentVal();

		//Print out information for the driver/debugger
		SmartDashboard::PutNumber("Yaw Gyro", drive->getYawGyro()->GetAngle());
		SmartDashboard::PutNumber("Pitch Gyro", drive->getPitchGyro()->GetAngle());
		SmartDashboard::PutNumber("Spatula Open Switch", spatula->GetOpenLimit()->Get());
		SmartDashboard::PutNumber("Spat Pos", spatula->GetEnco()->GetDistance());
		SmartDashboard::PutNumber("Spat Open Limit", spatula->GetOpenLimit()->Get());
		SmartDashboard::PutNumber("Lift Pos", lift->GetPot()->GetAverageValue());
		SmartDashboard::PutNumber("Lift Upper Bound Switch", lift->getUpperBound()->Get());
		SmartDashboard::PutNumber("Lift Lower Bound", lift->getLowerBound()->Get());
		SmartDashboard::PutNumber("Rake Prox Switch", rake->getDrawInSwitch()->Get());
	}

	void DisabledPeriodic()
	{
		drive->ResetAutoCorrect();
		drive->UpdateAdjustmentVal();
		SmartDashboard::PutNumber("Yaw Gyro", drive->getYawGyro()->GetAngle());
		SmartDashboard::PutNumber("Pitch Gyro", drive->getPitchGyro()->GetAngle());
		SmartDashboard::PutNumber("Spatula Open Switch", spatula->GetOpenLimit()->Get());
		SmartDashboard::PutNumber("Spat Pos", spatula->GetEnco()->GetDistance());
		SmartDashboard::PutNumber("Spat Open Limit", spatula->GetOpenLimit()->Get());
		SmartDashboard::PutNumber("Lift Pos", lift->GetPot()->GetAverageValue());
		SmartDashboard::PutNumber("Lift Upper Bound Switch", lift->getUpperBound()->Get());
		SmartDashboard::PutNumber("Lift Lower Bound", lift->getLowerBound()->Get());
		SmartDashboard::PutNumber("Rake Prox Switch", rake->getDrawInSwitch()->Get());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
