#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <cmath>
#include "SmartDashboard/SmartDashboard.h"
#include "RobotDrive.h"
#include "Gyro.h"
#include "DigitalInput.h"
#include "Ultrasonic.h"
#include "Timer.h"
#include "MORESubsystem.h"
#include "GlobalDefines.h"

#define GYRO_SENSITIVITY 0.0017

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
	//Ultrasonic sensor to tell the robot when it is close to the step
	Ultrasonic* sonic;
	//A timer to be used to keep track of how long the drive train moves
	Timer* autoTimer;

	//used to keep the robot at the same heading in teleop when its not turning
	float currentHeading;

	float GetTurnSpeed(float setPoint);
public:
	//Define the auto flags
	//this flag will tell the drive-train to drive for a specified time in autonomous
	static const int TIME = 1;
	//this flag will tell the drive-train to drive until the bump limit switches are pressed
	static const int BUMP = 2;
	//this flag will tell the drive-train to drive until we get close to the step according to the ultrasonic sensor
	static const int ULTRASONIC = 4;

	//Constructor/Deconstructor
	DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
			int lBumpLimitPort, int rBumpLimitPort, int ultrasonicPingPort, int ultrasonicEchoPort,
			int gyroPort, string name);
	~DriveTrain();

	void Drive(float x, float y, float rot);
	void TankDrive(float left, float right);

	//Accessor methods
	Gyro* getGyro(){return gyro;}
	double getAutoTimer(){return autoTimer->Get();}
	float GetCurrentHeading(){return currentHeading;}
Ultrasonic* GetUltrasonic(){return sonic;}

	//MORESubsystem autonomous functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};

#endif
