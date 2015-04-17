/*
 * Spatula.h
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#ifndef SPATULA_H
#define SPATULA_H

#define SPATULA_CLOSED_1 500
//#define SPATULA_OPEN_1 925

#define SPATULA_CLOSED_2 400//3242
//#define SPATULA_OPEN_2 3700

#define CURVE_IN -1.0
#define CURVE_OUT  1.0


#include "CANTalon.h"
#include "MORESubsystem.h"
#include "Encoder.h"
#include "DigitalInput.h"
#include "GlobalDefines.h"

class Spatula : public MORESubsystem
{
private:
	CANTalon* rotaryMotor;
	Encoder* enco;
	DigitalInput* openLimit;
	bool movingUp;
	bool movingDown;
	int spatClosedVal;
	//int spatOpenVal;
public:
	static const int OPEN_SPAT = 1;/**< this autonomous flag will tell the spatula to open*/
	/**
	 * The constructor for the Spatula class
	 *
	 * @param talonDeviceNumber The CAN device ID number for the TalonSRX that will control the spatula
	 * @param potPort The port of the potentiometer for the spatula
	 * @param name The name of the subsystem
	 */
	Spatula(int talonDeviceNumber, int encoAPort, int encoBPort, int openLimitPort, string robotNumber, string name);
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
	 * @return Returns an instance of the encoder
	 */
	Encoder* GetEnco(){return enco;}

	/**
	 * @return Returns an instance of the limit switch that tells the spatula if it is open
	 */
	DigitalInput* GetOpenLimit(){return openLimit;}

	///MORESubsystem Auto Functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};


#endif /* SRC_SPATULA_H_ */
