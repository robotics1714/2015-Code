/*
 * Spatula.h
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#ifndef SPATULA_H
#define SPATULA_H

#define SPATULA_UP 400
#define SPATULA_DOWN 1200


#define CURVE_IN -0.5
#define CURVE_OUT  0.5


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
	Spatula(int talonDeviceNumber, int potPort, string name);
	~Spatula();

	bool MoveUp();
	bool MoveDown();

	void StartMoveUp();
	void StartMoveDown();

	///Rotates the bottom of the lift
	void Rotate(float speed);

	AnalogInput* GetPot(){return pot;}

	///MORESubsystem Auto Functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};


#endif /* SRC_SPATULA_H_ */
