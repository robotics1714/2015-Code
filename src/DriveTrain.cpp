#include "DriveTrain.h"

DriveTrain::DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
		Gyro* scope, string name) : MORESubsystem(name)
{
	drive = new RobotDrive(frontLeftPort, rearLeftPort, frontRightPort, rearRightPort);
	gyro = scope;
	autoTimer = new Timer();
}

DriveTrain::~DriveTrain()
{
	delete drive;
	delete gyro;
}

void DriveTrain::Drive(float x, float y, float rot)
{
	drive->MecanumDrive_Cartesian(x, y, rot/*, gyro->GetAngle()*/);//Commented out gyro bc we don't have one
}

//param1: magnitude [-1, 1]
//param2: direction (degrees)
//param3: rotation [-1, 1]
//param4: time (seconds)
void DriveTrain::SetUpAuto(AutoInstructions instructions)
{
	//Reset and start on the timer
	autoTimer->Reset();
	autoTimer->Start();
}

//param1: magnitude [-1, 1]
//param2: direction (degrees)
//param3: rotation [-1, 1]
//param4: time (seconds)
int DriveTrain::Auto(AutoInstructions instructions)
{
	//Put the instruction parameters into more readable variables
	double magnitude = instructions.param1;
	double dir = instructions.param2;
	double rot = instructions.param3;
	double time = instructions.param4;

	//Drive with the specified instructions until the time has passed
	if(autoTimer->Get() <= time)
	{
		drive->MecanumDrive_Polar(magnitude, dir, rot);
		return (int)(true);
	}
	else
	{
		return (int)(false);
	}
}
