#include "DriveTrain.h"

DriveTrain::DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
		int lBumpLimitPort, int rBumpLimitPort, Gyro* scope, string name) : MORESubsystem(name)
{
	//Initialize the member classes of the drive train class
	drive = new RobotDrive(frontLeftPort, rearLeftPort, frontRightPort, rearRightPort);
	gyro = scope;
	gyro->SetSensitivity(GYRO_SENSITIVITY);
	leftBumpSwitch = new DigitalInput(lBumpLimitPort);
	rightBumpSwitch = new DigitalInput(rBumpLimitPort);
	autoTimer = new Timer();
	currentHeading = 0;

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

	//Multiply the gyro angle by -1 because it is backwards from the andymark gyro
	drive->MecanumDrive_Cartesian(x, y, rotation, gyro->GetAngle()*-1/*0.0*/);//Commented out gyro bc we don't have one
	//drive->MecanumDrive_Polar(0.5, 0, 0);
}

void DriveTrain::TankDrive(float left, float right)
{
	drive->TankDrive(left, right);
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
 *If both flags are set, the robot will drive with the set magnitude, direction, and rotation until
 *either of the above conditions for stopping are met
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

	//if we get here, something went wrong, so return false
	return false;
}

//This functions will return the turn speed needed to get the robot to a certain heading
float DriveTrain::GetTurnSpeed(float setPoint)
{
	float error = 0.0;
	float turnSpeed = 0.0;
	float kP = 0.06;//0.18;
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
	error = setPoint - myAngle;

	SmartDashboard::PutNumber("myAngle", myAngle);
	SmartDashboard::PutNumber("Current Heading", currentHeading);

	turnSpeed = (error * kP);

	return turnSpeed;
}
