/*
 * Spatula.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#include "Spatula.h"

Spatula::Spatula(int talonDeviceNumber, int potPort, string name) : MORESubsystem(name)
{
	rotaryMotor = new CANTalon(talonDeviceNumber);
	pot = new AnalogInput(potPort);
	movingUp=false;
	movingDown=false;
}

Spatula::~Spatula()
{
	delete rotaryMotor;
	delete pot;
}

///Set Motor Speed

void Spatula::Rotate(float speed)
{
	rotaryMotor->Set(speed);
}

void Spatula::StartMoveUp()
{
	if((!movingUp)&&(!movingDown))
	{
		movingUp=true;
	}
}

void Spatula::StartMoveDown()
{
	if((!movingUp)&&(!movingDown))
	{
		movingDown=true;
	}
}

bool Spatula::MoveUp()
{
	///Check if motor is moving up
	if(movingUp)
	{
		///Move up if the spatula isn't up already
		if(pot->GetAverageValue() < SPATULA_UP)
		{
			rotaryMotor->Set(CURVE_IN);
		}
		///Set Motor to 0
		else
		{
			rotaryMotor->Set(0);
			movingUp=false;
		}
	}
	return movingUp;
}

bool Spatula::MoveDown()
{
	///Check if Motor is moving down
	if(movingDown)
	{
		///Move down if the spatula isn't moving
		if(pot->GetAverageValue() < SPATULA_UP)
		{
			rotaryMotor->Set(CURVE_OUT);
		}
		///Set Motor to 0
		else
		{
			rotaryMotor->Set(0);
			movingDown=false;
		}
	}
	return movingDown;
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
