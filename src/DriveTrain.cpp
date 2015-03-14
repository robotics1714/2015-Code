#include "DriveTrain.h"
#include <fstream>

DriveTrain::DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
		int lBumpLimitPort, int rBumpLimitPort, int ultrasonicPingPort, int ultrasonicEchoPort,
		int yawGyroPort, int pitchGyroPort, string name) : MORESubsystem(name)
{
	//Initialize the member classes of the drive train class
	drive = new RobotDrive(frontLeftPort, rearLeftPort, frontRightPort, rearRightPort);
	yawGyro = new Gyro(yawGyroPort);
	yawGyro->SetDeadband(0.001);
	yawGyro->SetSensitivity(GYRO_SENSITIVITY);
	//lastLoopHeading = 0;
	pitchGyro = new Gyro(pitchGyroPort);
	pitchGyro->SetDeadband(0.005);
	pitchAngleAdjustmentVal = 0;
	leftBumpSwitch = new DigitalInput(lBumpLimitPort);
	rightBumpSwitch = new DigitalInput(rBumpLimitPort);
	sonic = new Ultrasonic(ultrasonicPingPort, ultrasonicEchoPort);
	sonic->SetAutomaticMode(true);
	autoTimer = new Timer();
	tipTimer = new Timer();
	tipTimer->Reset();
	currentHeading = 0;
	lastRampUpAutoOutput = 0.25;//Start the ramp up at 25%

	drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
}

DriveTrain::~DriveTrain()
{
	delete drive;
	delete yawGyro;
	delete leftBumpSwitch;
	delete rightBumpSwitch;
	delete autoTimer;
}

void DriveTrain::Drive(float x, float y, float rot)
{
	float rotation = rot;
	float heading = (yawGyro->GetAngle()*-1)-180;
	float tiltSpeed = GetAntiTiltSpeed();

	//If the driver wants to turn, make the current angle the current heading and turn that much
	if(fabs(rot) > 0)
	{
		currentHeading = yawGyro->GetAngle();
		rotation = rot;
	}
	//If the driver does not want to turn, make sure the robot stays and the desired heading
	else
	{
		//Normalize the current heading between [-180, 180)
		/*while(currentHeading >= 180)
		{
			currentHeading -= 360;
		}
		while(currentHeading <= -180)
		{
			currentHeading += 360;
		}*/
		//Find the appropriate amount of turn
		rotation = GetTurnSpeed(currentHeading);
	}
	SmartDashboard::PutNumber("Current Heading", currentHeading);

	//If the robot is tilting, ignore the driver's instructions and un-tilt it
	if(tiltSpeed > 0)//Basically checking if it equals 0
	{
		x = 0;
		y = tiltSpeed;
		rotation = 0;
		heading = 0;//If this was set to the gyro, the robot might move in a funky direction instead of backwards/forwards relative to itself
	}

	//Multiply the gyro angle by -1 because it is backwards from the andymark gyro
	drive->MecanumDrive_Cartesian(x, y, rotation, heading/*0.0*/);
	//drive->MecanumDrive_Polar(0.5, 0, 0);

	SmartDashboard::PutNumber("Ultrasonic:", sonic->GetRangeInches());
}

void DriveTrain::TankDrive(float left, float right)
{
	drive->TankDrive(left, right);
}

void DriveTrain::ResetAutoCorrect()
{
	yawGyro->Reset();
	currentHeading = yawGyro->GetAngle();
}

void DriveTrain::ResetAntiTip()
{
	pitchAngleAdjustmentVal = pitchGyro->GetAngle();
}
//Because the pitch gyro changed when the robot is rotated, we will use this to check if the robot rotated
//and reset the pitch gyro if it is
/*void DriveTrain::CorrectPitchGyro()
{
	if(fabs(lastLoopHeading - yawGyro->GetAngle()) > 0.5)
	{
		pitchGyro->Reset();
	}
	lastLoopHeading = yawGyro->GetAngle();
}*/

//param1: magnitude [-1, 1]
//param2: direction (degrees)
//param3: where to rotate to (degrees)
//param4: time (seconds)
void DriveTrain::SetUpAuto(AutoInstructions instructions)
{
	//Reset and start on the timer
	autoTimer->Reset();
	autoTimer->Start();

	//Start the ramp up at 25%
	lastRampUpAutoOutput = 0.25;
}

/*param1: magnitude [-1, 1]
 *param2: direction (degrees)
 *param3: where to rotate to (degrees)
 *param4: time (seconds)
 *
 *If the TIME flag is set, the robot will drive with the set magnitude, direction, and rotation until
 *the set time
 *
 *If the BUMP flag is set, the robot will drive with the set magnitude, direction, and rotation until
 *one of the two bump limit switches are pressed
 *
 */
int DriveTrain::Auto(AutoInstructions instructions)
{
	//Put the instruction parameters into more readable variables
	double magnitude = instructions.param1;
	double dir = instructions.param2;
	double rot = instructions.param3;//Where to robot should rotate to
	double time = instructions.param4;
	double turnSpeed = GetTurnSpeed(rot);

	//If the robot it tilting, do no action other than fixing the tilt
	double tiltSpeed = GetAntiTiltSpeed();
	if(tiltSpeed > 0)
	{
		magnitude = tiltSpeed;
		dir = 0;//Straight relative to the robot
		turnSpeed = 0;//Don't want turning to mess anything up
	}

	//Drive with the specified instructions until the time has passed
	if((instructions.flags & TIME) == TIME)
	{
		if(autoTimer->Get() <= time)
		{
			drive->MecanumDrive_Polar(magnitude, dir, turnSpeed);
			return (int)(true);
		}
		else
		{
			drive->MecanumDrive_Polar(0, 0, 0);
			return (int)(false);
		}
	}
	//Drive with the specified instructions until one of the two bump switches are pressed
	if((instructions.flags & BUMP) == BUMP)
	{
		if((leftBumpSwitch->Get() == RELEASED) && (rightBumpSwitch->Get() == RELEASED))
		{
			drive->MecanumDrive_Polar(magnitude, dir, turnSpeed);
			return (int)(true);
		}
		else
		{
			drive->MecanumDrive_Polar(0, 0, 0);
			return (int)(false);
		}
	}
	//Drive until the unltrasonic sensor says we are within 5 inches or either of the limit switches are pressed
	if((instructions.flags & ULTRASONIC) == ULTRASONIC)
	{
		double approachSetPoint = 5.0;
		double departureSetPoint = 48;
		double curLoc = sonic->GetRangeInches();
		double error, speed;
		double kP = 0.016;
		//If we are driving back
		if((magnitude < 0) && (curLoc > approachSetPoint) && (leftBumpSwitch->Get() == RELEASED) &&
				(rightBumpSwitch->Get() == RELEASED) && (autoTimer->Get() < time))
		{
			error = curLoc - approachSetPoint;
			speed = error * kP * magnitude;
			if(fabs(speed) > fabs(magnitude))
			{
				speed = magnitude;
			}
			drive->MecanumDrive_Polar(speed, dir, turnSpeed);
			SmartDashboard::PutNumber("Auto Speed:", speed);
			return (int)(true);
		}
		//If we are going forward
		else if((magnitude > 0) && (curLoc < departureSetPoint) && (autoTimer->Get() < time))
		{
			kP = 0.008;
			error = curLoc;
			speed = error * kP * magnitude;
			//Set a floor and ceiling for the speed
			if(fabs(speed) > fabs(magnitude))
			{
				speed = magnitude;
			}
			if(speed < 0.15)
			{
				speed = 0.15;
			}
			drive->MecanumDrive_Polar(speed, dir, turnSpeed);
			SmartDashboard::PutNumber("Auto Speed:", speed);
			return (int)(true);
		}
		else
		{
			drive->MecanumDrive_Polar(0, 0, 0);
			//Save the time
			/*ofstream file("/home/lvuser/time.csv", ofstream::app);
			file<<kP << "," <<autoTimer->Get();
			if(rightBumpSwitch->Get() == PRESSED)
			{
				file<<",Right limit";
			}
			if(leftBumpSwitch->Get() == PRESSED)
			{
				file<<",Left limit";
			}
			if(sonic->GetRangeInches() < 5)
			{
				file<<",Ultrasonic";
			}
			file<<endl;
			file.close();*/
			return (int)(false);
		}
	}
	if((instructions.flags & RAMP_UP) == RAMP_UP)
	{
		if(autoTimer->Get() <= time)
		{
			float k = 0.02;
			//This is the first of difference sort
			float adjustedSpeed = lastRampUpAutoOutput + (k * (magnitude - lastRampUpAutoOutput));
			//Update the lastRampUpAutoOutput
			lastRampUpAutoOutput = adjustedSpeed;
			//Move
			drive->MecanumDrive_Polar(adjustedSpeed, dir, turnSpeed);
			return (int)(true);
		}
		else
		{
			drive->MecanumDrive_Polar(0, 0, 0);
			return (int)(false);
		}
	}
	SmartDashboard::PutNumber("AutoTimer", autoTimer->Get());
	//if we get here, something went wrong, so return false
	return (int)(false);
}

//This functions will return the turn speed needed to get the robot to a certain heading
float DriveTrain::GetTurnSpeed(float setPoint)
{
	float error = 0.0;
	float turnSpeed = 0.0;
	float kP = 0.0175;
	//Put my angle into a range of [-180, 180]
	float myAngle = yawGyro->GetAngle();
	/*while(myAngle >= 180)
	{
		myAngle -= 360;
	}
	while(myAngle <= -180)
	{
		myAngle += 360;
	}*/

	//Calculate the error
	error = myAngle - setPoint;

	SmartDashboard::PutNumber("myAngle", myAngle);
	SmartDashboard::PutNumber("Current Heading", currentHeading);

	turnSpeed = (error * kP);

	SmartDashboard::PutNumber("turn speed", turnSpeed);

	return turnSpeed;
}

void DriveTrain::UpdateAdjustmentVal()
{
	float difference = pitchGyro->GetAngle() - pitchAngleAdjustmentVal;
	pitchAngleAdjustmentVal += difference * 0.01;
	SmartDashboard::PutNumber("Pitch Adjustment Value", pitchAngleAdjustmentVal);
	SmartDashboard::PutNumber("Will Anti-Tip at:", -7.5 + pitchAngleAdjustmentVal);
}

//Will return 0 if the robot is in danger of tipping, otherwise return the speed in which the robot should go to avoid tipping
float DriveTrain::GetAntiTiltSpeed()
{
	float speed;
	float pitchAngle = pitchGyro->GetAngle();

	//If the robot's tilt is equal to or greater than 35 degrees tilting back and we still have hope of correcting
	//ourselves, drive backwards to correct it
	if(pitchAngle <= (-7.5 + pitchAngleAdjustmentVal) && tipTimer->Get() <= TIP_CORRECTION_LIMIT)
	{
		speed = 0.75;
		tipTimer->Start();
	}
	//If the robot's tilt is equal to or greater than 45 degrees tilting forwards and we still have hope of correcting
	//ourselves, drive backwards to correct it
	/*else if(pitchAngle <= -45 && tipTimer->Get() <= TIP_CORRECTION_LIMIT)
	{
		speed = 0.75;
		tipTimer->Start();
	}*/
	//Otherwise, the robot is in good shape (or we gave up on saving it), so don't do anything
	else
	{
		speed = 0;
	}

	//If the robot is at an untippy pitch, stop and reset the timer
	if(/*pitchAngle > 45 &&*/ pitchAngle > (-7.5 + pitchAngleAdjustmentVal))
	{
		tipTimer->Stop();
		tipTimer->Reset();
	}

	return speed;
}
