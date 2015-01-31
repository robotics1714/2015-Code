/*
 * Rake.h
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#ifndef RAKE_H
#define RAKE_H

#include "MORESubsystem.h"
#include "GlobalDefines.h"
#include "CANTalon.h"
#include "DigitalInput.h"
#include "Solenoid.h"

class Rake : public MORESubsystem
{
private:
	CANTalon* drawIn;
	Solenoid* actuateSolenoid;
	//Timer* moveTimer;
	DigitalInput* leftDrawInSwitch;
	DigitalInput* rightDrawInSwitch;
	float drawInSpeed;
	bool drawingIn;
	/*float moveTimeDuration;
	bool movingForTime;*/
public:
	/**
	 *  Constructor for the Rake class.
	 *  @param talonDeviceNumber The CAN device ID number for the TalonSRX that will control the rake.
	 *  @param solenoidPortNumber The port on the PCM that the solenoid for actauting the rake is plugged in to
	 *  @param leftLimitPort The limit switch that checks if the left side of the rake is in
	 *  @param rightLimitPort The limit switch that check if the ride side of the rake is in
	 *  @param name subsystem name.
	 */
	Rake(int talonDeviceNumber, int solenoidPortNumber, int leftLimitPort, int rightLimitPort, string name);
	/**
	 * Deconstructor for the Rake class,
	 */
	~Rake();

	/**
	* Moves the extensions on the rake in/out
	*
	* @param speed value of the speed that the extensions are moving
	* @return returns false when the extension hits either limit switch and returns true when its moving.
	*/
	bool Move(float speed);

	///Starts the movement timer
	/**
	 * Draws in the extensions on the rake
	 *
	 * @param speed The speed that the extensions move in at
	 */
	void StartDrawIn(float speed);

	/**
	 * Move the Rake for time in seconds
	 * @retun Returns true when the extensions are moving in and false when they are done moving in
	 */
	bool DrawIn();

	void MoveUp();
	void MoveDown();

	/**
	 * Stops the Rake Motor
	 */
	void Stop();


	///MORESubsystem Auto Instructions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};


#endif /* SRC_RAKE_H_ */
