#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "MORESubsystem.h"
#include "RobotDrive.h"
#include "Gyro.h"
#include "Timer.h"

class DriveTrain : public MORESubsystem
{
private:
	RobotDrive* drive;
	Gyro* gyro;
	Timer* autoTimer;

public:
	//Define the auto flags
	//None as of now

	//Constructor/Deconstructor
	DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort, Gyro* scope, string name);
	~DriveTrain();

	void Drive(float x, float y, float rot);

	//Accessor methods
	Gyro* getGyro(){return gyro;}

	//MORESubsystem autonomous functions
	void SetUpAuto(AutoInstructions instructions);
	int Auto(AutoInstructions instructions);
};

#endif
