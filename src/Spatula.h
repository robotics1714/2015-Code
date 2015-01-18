/*
 * Spatula.h
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#ifndef SPATULA_H
#define SPATULA_H

#define SPATULA_UP 1
#define SPATULA_DOWN 0

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
public:
	static const int CURVE_IN = 1;
	static const int CURVE_OUT = -1;

	Spatula(int talonDeviceNumber, int potPort, string name);
	~Spatula();

	bool MoveUp();
	bool MoveDown();

	void StartMoveUp();
	void StartMoveDown();

	///Rotates the bottom of the lift
	void Rotate(float speed);


	///MORESubsystem Auto Functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};


#endif /* SRC_SPATULA_H_ */
