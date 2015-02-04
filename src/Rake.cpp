/*
 * Rake.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#include "Rake.h"

Rake::Rake(int talonDeviceNumber, int solenoidPortNumber, int drawInPort, string name) : MORESubsystem(name)
{
	drawIn = new CANTalon(talonDeviceNumber);
	actuateSolenoid = new Solenoid(solenoidPortNumber);
	drawInSwitch = new DigitalInput(drawInPort);
	drawInSpeed = 0;
	drawingIn = false;
}

Rake::~Rake()
{
	delete drawIn;
	delete actuateSolenoid;
	delete drawInSwitch;
}

bool Rake::Move(float speed)
{
	//The draw in switch is activated, stop moving the rake in
	if((drawInSwitch->Get() == PRESSED))
	{
		drawIn->Set(0);
		return false;
	}
	else
	{
		drawIn->Set(speed);
		return true;
	}
}

void Rake::StartDrawIn(float speed)
{
	drawingIn = true;
	drawInSpeed = speed;
}

bool Rake::DrawIn()
{
	if(drawingIn)
	{
		if(!Move(drawInSpeed))
		{
			drawingIn = false;
		}
	}

	return drawingIn;
}

void Rake::MoveUp()
{
	actuateSolenoid->Set(true);
}

void Rake::MoveDown()
{
	actuateSolenoid->Set(false);
}

/*void Rake::StartMoveForTime(float speed, float time)
{
	//Make sure the program is not already moving for time
	if(!movingForTime)
	{
		//Reset the timers and set the variables for the MoveForTime function
		moveTimer->Reset();
		moveTimer->Start();
		moveTimeSpeed = speed;
		moveTimeDuration = time;
		movingForTime = true;
	}

}

bool Rake::MoveForTime()
{
	if(movingForTime)
	{
		if(moveTimer->Get() < moveTimeDuration)
		{
			//If the limit switch is pressed, stop moving
			if(!Move(moveTimeSpeed))
			{
				Move(0);
				movingForTime = false;
				moveTimeSpeed = 0;
				moveTimeDuration = 0;
			}
		}
		else
		{
			Move(0);
			movingForTime = false;
			moveTimeSpeed = 0;
			moveTimeDuration = 0;
		}
	}

	return movingForTime;
}*/

void Rake::Stop()
{
	drawingIn = false;
	drawInSpeed = 0;
	Move(0);

}

//param1: speed
//param2-param4: unused
void Rake::SetUpAuto(AutoInstructions instructions)
{
	float speed = instructions.param1;

	StartDrawIn(speed);
}

int Rake::Auto(AutoInstructions instructions)
{
	return (int)(DrawIn());
}
