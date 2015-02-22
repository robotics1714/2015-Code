/*
 * Spatula.h
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#ifndef SPATULA_H
#define SPATULA_H

#define SPATULA_CLOSED_1 774
#define SPATULA_OPEN_1 1300

#define SPATULA_CLOSED_2 3280//3242
#define SPATULA_OPEN_2 3700

#define CURVE_IN -1.0
#define CURVE_OUT  1.0


#include "CANTalon.h"
#include "MORESubsystem.h"
#include "AnalogInput.h"

class Spatula : public MORESubsystem
{
private:
	CANTalon* rotaryMotor;
	AnalogInput* pot;
	bool movingUp;
	bool movingDown;
	int spatClosedVal;
	int spatOpenVal;
public:
	/**
	 * The constructor for the Spatula class
	 *
	 * @param talonDeviceNumber The CAN device ID number for the TalonSRX that will control the spatula
	 * @param potPort The port of the potentiometer for the spatula
	 * @param name The name of the subsystem
	 */
	Spatula(int talonDeviceNumber, int potPort, string robotNumber, string name);
	/**
	 * The deconstructor for the Spatula class
	 */
	~Spatula();

	/**
	 * Moves the spatula to the all the way up position
	 *
	 * @return Returns true when the spatula is moving up. Returns false when it's done moving up.
	 */
	bool MoveUp();
	/**
	 * Moves the spatula to the all the way down position
	 *
	 * @return Returns true when the spatula is moving down. Returns false when it's done moving down.
	 */
	bool MoveDown();

	/**
	 * Allows the MoveUp function to start
	 */
	void StartMoveUp();
	/**
	 * Allows the MoveDown function to start
	 */
	void StartMoveDown();

	/**
	 * Rotates the spatula in or out at a specified speed
	 *
	 * @param speed The speed and direction to move the spatula
	 */
	void Rotate(float speed);

	/**
	 * Stops the spatula
	 */
	void Stop();

	/**
	 * @return Returns an instance of the potentiometer
	 */
	AnalogInput* GetPot(){return pot;}

	///MORESubsystem Auto Functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};


#endif /* SRC_SPATULA_H_ */
