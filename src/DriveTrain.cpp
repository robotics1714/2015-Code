#include "DriveTrain.h"

DriveTrain::DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
		Gyro* scope, string name) : MORESubsystem(name)
{
	drive = new RobotDrive(frontLeftPort, rearLeftPort, frontRightPort, rearRightPort);
	gyro = scope;
}

DriveTrain::~DriveTrain()
{
	delete drive;
	delete gyro;
}

void DriveTrain::Drive(float x, float y, float rot)
{
	drive->MecanumDrive_Cartesian(x, y, rot, gyro->GetAngle());
}

void DriveTrain::SetUpAuto(AutoInstructions instructions)
{
	//TODO fill in this
}

int DriveTrain::Auto(AutoInstructions instructions)
{
	//TODO fill this in
}
