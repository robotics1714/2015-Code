/*
 * Rake.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#include "Rake.h"

Rake::Rake(int talonDeviceNumber, string name) : MORESubsystem(name)
{
	winch = new CANTalon(talonDeviceNumber);
	moveTimer = new Timer();
	moveTimeSpeed = 0;
	moveTimeDuration = 0;
	movingForTime = false;
}

Rake::~Rake()
{
	delete winch;
	delete moveTimer;
}

void Rake::Move(float speed)
{
	winch->Set(speed);
}

void Rake::StartMoveForTime(float speed, float time)
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
			winch->Set(moveTimeSpeed);
		}
		else
		{
			winch->Set(0);
			movingForTime = false;
			moveTimeSpeed = 0;
			moveTimeDuration = 0;
		}
	}

	return movingForTime;
}

void Rake::Stop()
{
	movingForTime = false;
	moveTimeSpeed = 0;
	moveTimeDuration = 0;
	winch->Set(0);

}

//param1: speed
//param2: duration
//param3-param4: unused
void Rake::SetUpAuto(AutoInstructions instructions)
{
	float speed = instructions.param1;
	float duration = instructions.param2;

	StartMoveForTime(speed, duration);
}

int Rake::Auto(AutoInstructions instructions)
{
	return (int)(MoveForTime());
}
