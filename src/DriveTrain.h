#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "RobotDrive.h"
#include "Gyro.h"
#include "DigitalInput.h"
#include "Timer.h"
#include "MORESubsystem.h"
#include "GlobalDefines.h"

class DriveTrain : public MORESubsystem
{
private:
	//The class that moves the drivetrain
	RobotDrive* drive;
	//The gyro to keep track of the robot heading for field centric driving
	Gyro* gyro;
	//The limit switches to tell the robot when it hits the step in autonomous
	DigitalInput* leftBumpSwitch;
	DigitalInput* rightBumpSwitch;
	//A timer to be used to keep track of how long the drive train moves
	Timer* autoTimer;

public:
	//Define the auto flags
	//this flag will tell the drive-train to drive for a specified time in autonomous
	static const int TIME = 1;
	//this flag will tell the drive-train to drive until the bump limit switches are pressed
	static const int BUMP = 2;

	//Constructor/Deconstructor
	DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
			int lBumpLimitPort, int rBumpLimitPort, Gyro* scope, string name);
	~DriveTrain();

	void Drive(float x, float y, float rot);

	//Accessor methods
	Gyro* getGyro(){return gyro;}
	double getAutoTimer(){return autoTimer->Get();}

	//MORESubsystem autonomous functions
	void SetUpAuto(AutoInstructions instructions);
	int Auto(AutoInstructions instructions);
};

#endif
