/*
 * Spatula.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#include "Spatula.h"

Spatula::Spatula(int talonDeviceNumber, int encoAPort, int encoBPort, int openLimitPort,
		string robotNumber, string name) : MORESubsystem(name)
{
	rotaryMotor = new CANTalon(talonDeviceNumber);
	enco = new Encoder(encoAPort, encoBPort);
	openLimit = new DigitalInput(openLimitPort);
	movingUp=false;
	movingDown=false;

	//Use these values for the practice bot
	if(robotNumber == "2")
	{
		spatClosedVal = SPATULA_CLOSED_2;
		//spatOpenVal = SPATULA_OPEN_2;
	}
	//Use these values for the competition bot
	else
	{
		spatClosedVal = SPATULA_CLOSED_1;
		//spatOpenVal = SPATULA_OPEN_1;
	}
}

Spatula::~Spatula()
{
	delete rotaryMotor;
	delete enco;
	delete openLimit;
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
		if(enco->GetDistance() > spatClosedVal)
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
		///Move down until the spatula hits the limit switch
		if(openLimit->Get() == RELEASED)
		{
			rotaryMotor->Set(CURVE_OUT);
		}
		///Set Motor to 0
		else
		{
			rotaryMotor->Set(0);
			//Because we are at our "home", reset the encoder
			enco->Reset();
			movingDown=false;
		}
	}
	return movingDown;
}

void Spatula::Stop()
{
	rotaryMotor->Set(0);
	movingUp = false;
	movingDown = false;
}

void Spatula::SetUpAuto(AutoInstructions instructions)
{
	if((instructions.flags & OPEN_SPAT) == OPEN_SPAT)
	{
		StartMoveDown();
	}
}

int Spatula::Auto(AutoInstructions instructions)
{
	if((instructions.flags & OPEN_SPAT) == OPEN_SPAT)
	{
		return (int)MoveDown();
	}
	return 0;
}
