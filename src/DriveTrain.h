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

/**
 *  This class controls the functionality of the mecanum DriveTrain.
 */
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
	static const int TIME = 1; /**< this autonomous flag will tell the drive-train to drive for a specified time in autonomous */
	static const int BUMP = 2; /**< this autonomous flag will tell the drive-train to drive for a specified time in autonomous*/
	static const int ULTRASONIC = 4; /**< this autonomous flag will tell the drive-train to drive until we get close to the step according to the ultrasonic sensor*/

	/**
	 * The constructor for the DriveTrain class.
	 * @param frontLeftPort the port for the motor controller for the front Left drive motor.
	 * @param rearLeftPort the port for the motor controller for the rear Left drive motor.
	 * @param frontRightPort the port for the motor controller for the front Right drive motor.
	 * @param rearRightPort the port for the motor controller for the rear Right drive motor.
	 * @param lBumpLimitPort the port for the left dorsal limit switch for detecting the step.
	 * @param rBumpLimitPort the port for the right dorsal limit switch for detecting the step.
	 * @param ultrasonicPingPort the port for the ultrasonic speaker for detecting the step.
	 * @param ultrasonicEchoPort the port for the ultrasonic receiver for detecting the step.
	 * @param gyroPort the port for the gyro to keep the robot at constant heading and for field centric capabilities.
	 * @param name the name of the subsystem.
	 *
	 */
	DriveTrain(int frontLeftPort, int rearLeftPort, int frontRightPort, int rearRightPort,
			int lBumpLimitPort, int rBumpLimitPort, int ultrasonicPingPort, int ultrasonicEchoPort,
			int gyroPort, string name);
	/**
	 * The Deconstructor for the DriveTrain class.
	 */
	~DriveTrain();
	/**
	 *  Makes the robot drive in mecanum, strafing fashion.
	 *  @param x the value for the mecanum drive to move in the horizontal (x-axis) direction.
	 *  @param y the value for mecanum drive to move in forward or reverse (y-axis) directions.
	 *  @param rot the value for the mecanum drive to rotate heading on the robots axis.
	 */
	void Drive(float x, float y, float rot);
	/**
	 * Makes the robot drive in tank drive fashion.
	 * @param left the value for the tank drive to move the left side drive motors.
	 * @param right the value for the tank drive to move the right side drive motors.
	 */
	void TankDrive(float left, float right);


	//Accessor methods
	/**
	 * getGyro returns the value of the gyro at an instance.
	 */
	Gyro* getGyro(){return gyro;}
	/**
	 * getAutoTimer returns the value of the AutoTimer at an instance.
	 */
	double getAutoTimer(){return autoTimer->Get();}
	/**
	 * GetCurrentHeading returns the value of the heading at an instance.
	 */
	float GetCurrentHeading(){return currentHeading;}
	/**
	 * GetUltrasonic returns the value of the Ultrasonic sensor at an instant.
	 */
	Ultrasonic* GetUltrasonic(){return sonic;}

	/**
	 * @return Returns an instance of the left limit switch
	 */
	DigitalInput* GetLeftLimit(){return leftBumpSwitch;}
	/**
	 * @return Returns an instance of the right limit switch
	 */
	DigitalInput* GetRightLimit(){return rightBumpSwitch;}

	/**
	 * SetUpAuto sets up the autonomous steps.
	 * @param instructions param1: magnitude [-1, 1] param2: direction (degrees) param3: where to rotate to (degrees) param4: time (seconds).
	 */
	void SetUpAuto(AutoInstructions instructions) override;
	/**
	 * Auto autonomous instructions
	 * @param instructions If the TIME flag is set, the robot will drive with the set magnitude, direction, and rotation until the set time.
     * If the BUMP flag is set, the robot will drive with the set magnitude, direction, and rotation until
     * one of the two bump limit switches are pressed. If the ultrasonic flag is set, the robot will drive with set magnitude, direction, and rotation
     * until one limit switch reads a set value. param1: magnitude [-1, 1] param2: direction (degrees) param3: where to rotate to (degrees) param4: time (seconds).
	 *
	 */
	int Auto(AutoInstructions instructions) override;
};

#endif
