/*
 * Spatula.h
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#ifndef SPATULA_H
#define SPATULA_H

#include "CANTalon.h"
#include "MORESubsystem.h"

class Spatula : public MORESubsystem
{
private:
	CANTalon* rotaryMotor;
public:
	Spatula(int talonDeviceNumber, string name);
	~Spatula();

	///Rotates the bottom of the lift
	void Rotate(float speed);


	///MORESubsystem Auto Functions
	void SetUpAuto(AutoInstructions instructions);
	int Auto(AutoInstructions instructions);
};


#endif /* SRC_SPATULA_H_ */
