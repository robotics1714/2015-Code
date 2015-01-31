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
#include "Timer.h"

class Rake : public MORESubsystem
{
private:
	CANTalon* winch;
	Timer* moveTimer;
	DigitalInput* upperLimit;
	float moveTimeSpeed;
	float moveTimeDuration;
	bool movingForTime;
public:
	/**
	 *  Constructor for the Rake class.
	 *  @param talonDeviceNumber The CAN device ID number for the TalonSRX that will control the rake.
	 *  @param limitPort The upperbound limit switch that will stop the rake when it reaches the top.
	 *  @param name subsystem name.
	 */
	Rake(int talonDeviceNumber, int limitPort, string name);
	/**
	 * Deconstructor for the Rake class,
	 */
	~Rake();

	///Speed of the winch
	/**
	* Moves the rake at set speed
	*
	* @param speed value of the speed that the Rake is moving
	* @return returns false when the Rake hits the top limit switch and returns true when its moving.
	*/
	bool Move(float speed);

	///Starts the movement timer
	/**
	 * Allows for the Rake to Move For time
	 * @param speed the values of the speed that the Rake is moving
	 * @param time the time in seconds that the Rake is moving
	 */
	void StartMoveForTime(float speed, float time);

	/**
	 * Move the Rake for time in seconds
	 * @retun returns true when the Rake id moving and returns false when the Rake is done moving.
	 */
	bool MoveForTime();

	/**
	 * Stops the Rake Motor
	 */
	void Stop();


	///MORESubsystem Auto Instructions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};


#endif /* SRC_RAKE_H_ */
