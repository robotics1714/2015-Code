/*
 * Rake.cpp
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#include "Rake.h"

Rake::Rake(int talonDeviceNumber, int upSolenoidPortNumber, int downSolenoidPortNumber, int drawInPort, string name) : MORESubsystem(name)
{
	drawIn = new CANTalon(talonDeviceNumber);
	actuateSolenoid = new DoubleSolenoid(upSolenoidPortNumber, downSolenoidPortNumber);
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
	if((drawInSwitch->Get() == 1) && (speed < 0))//The proximity switch is opposite from limit switches, so when it is present, the sensor returns 1
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
	actuateSolenoid->Set(DoubleSolenoid::kForward);
}

void Rake::MoveDown()
{
	actuateSolenoid->Set(DoubleSolenoid::kReverse);
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
	if((instructions.flags & AUTO_DRAW_IN) == AUTO_DRAW_IN)
	{
		float speed = instructions.param1;

		StartDrawIn(speed);
	}
}

int Rake::Auto(AutoInstructions instructions)
{
	if((instructions.flags & AUTO_DRAW_IN) == AUTO_DRAW_IN)
	{
		return (int)(DrawIn());
	}
	if((instructions.flags & AUTO_MOVE_UP) == AUTO_MOVE_UP)
	{
		MoveUp();
		//Return false right away because the MoveUp function only needs to be called once to work
		return (int)false;
	}
	if((instructions.flags & AUTO_MOVE_DOWN) == AUTO_MOVE_DOWN)
	{
		MoveDown();
		//Return false right away because the MoveDown function only needs to be called once to work
		return (int)false;
	}

	//If we got here, something went wrong, so return false
	return (int)false;
}
