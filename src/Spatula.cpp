/*
 * Spatula.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#include "Spatula.h"

Spatula::Spatula(int talonDeviceNumber, string name) : MORESubsystem(name)
{
	rotaryMotor = new CANTalon(talonDeviceNumber);
}

Spatula::~Spatula()
{
	delete rotaryMotor;
}

///Set Motor Speed

void Spatula::Rotate(float speed)
{
	rotaryMotor->Set(speed);
}

void Spatula::SetUpAuto(AutoInstructions instructions)
{
	///TODO Fill this in
}

int Spatula::Auto(AutoInstructions instructions)
{
	///TODO fill this in
	return 0;
}
