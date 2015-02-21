#include "DriveTrain.h"
#include <fstream>

DriveTrain::DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
		int lBumpLimitPort, int rBumpLimitPort, int ultrasonicPingPort, int ultrasonicEchoPort,
		int gyroPort, string name) : MORESubsystem(name)
{
	//Initialize the member classes of the drive train class
	drive = new RobotDrive(frontLeftPort, rearLeftPort, frontRightPort, rearRightPort);
	gyro = new Gyro(gyroPort);
	gyro->SetDeadband(0.001);
	gyro->SetSensitivity(GYRO_SENSITIVITY);
	leftBumpSwitch = new DigitalInput(lBumpLimitPort);
	rightBumpSwitch = new DigitalInput(rBumpLimitPort);
	sonic = new Ultrasonic(ultrasonicPingPort, ultrasonicEchoPort);
	sonic->SetAutomaticMode(true);
	autoTimer = new Timer();
	currentHeading = 0;
	lastRampUpAutoOutput = 0;//Start the ramp up at 25%

	drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
}

DriveTrain::~DriveTrain()
{
	delete drive;
	delete gyro;
	delete leftBumpSwitch;
	delete rightBumpSwitch;
	delete autoTimer;
}

void DriveTrain::Drive(float x, float y, float rot)
{
	float rotation = rot;

	//If the driver wants to turn, make the current angle the current heading and turn that much
	if(fabs(rot) > 0)
	{
		currentHeading = gyro->GetAngle();
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

	//Multiply the gyro angle by -1 because it is backwards from the andymark gyro
	drive->MecanumDrive_Cartesian(x, y, rotation, (gyro->GetAngle()*-1)-180/*0.0*/);//Commented out gyro bc we don't have one
	//drive->MecanumDrive_Polar(0.5, 0, 0);

	SmartDashboard::PutNumber("Ultrasonic:", sonic->GetRangeInches());
}

void DriveTrain::TankDrive(float left, float right)
{
	drive->TankDrive(left, right);
}

void DriveTrain::ResetAutoCorrect()
{
	gyro->Reset();
	currentHeading = gyro->GetAngle();
}

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
	float myAngle = gyro->GetAngle();
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
