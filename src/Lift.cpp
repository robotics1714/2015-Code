#include "Lift.h"

Lift::Lift(int talonDeviceNumber, int encoAPort, int encoBPort, int upperBoundPort, int lowerBoundPort, string name) : MORESubsystem(name)
{
	liftMotor = new CANTalon(talonDeviceNumber);
	liftEncoder = new Encoder(encoAPort, encoBPort);
	liftEncoder->Reset();
	upperBound = new DigitalInput(upperBoundPort);
	lowerBound = new DigitalInput (lowerBoundPort);
	movingUpLevel = false;
	movingDownLevel = false;
	currentLevel = 0;
	levelEncoValues[0] = 0;
	levelEncoValues[1] = 10;
	levelEncoValues[2] = 100;
	levelEncoValues[3] = 1000;
	levelEncoValues[4] = 10000;
	levelEncoValues[5] = 100000;
	levelEncoValues[6] = 1000000;
}

Lift::~Lift()
{
	delete liftMotor;
	delete liftEncoder;
	delete upperBound;
	delete lowerBound;

}

void Lift::Move(float speed)
{
	//Make sure we stop when we hit a limit switch
	if( ((speed > 0) && (upperBound->Get() == RELEASED))  || ((speed < 0) && lowerBound->Get() == RELEASED) )
	{
		liftMotor->Set(speed);
	}
	else
	{
		liftMotor->Set(0);
	}
}
void Lift::StartMoveUpLevel()
{
	if(!movingUpLevel && !movingDownLevel && (currentLevel +1<7))
	{
		movingUpLevel = true;
		currentLevel = currentLevel +1;
	}
}

bool Lift::MoveUpLevel()
{
	//Check if movingUpLevel is true
	if (movingUpLevel)
	{
		//check if our position is less than the level we want to move to and the top limit switch is not pressed
		//if ^ is true move lift up if ^ is false stop motor and set movinguplevel to false
		if ((liftEncoder->GetDistance() < levelEncoValues[currentLevel]) && upperBound->Get() == RELEASED)
		{
			liftMotor->Set(FULL_SPEED_UP);
		}
		else
		{
			liftMotor->Set(0);
			movingUpLevel = false;
		}
	}
	return movingUpLevel;
}

void Lift::StartMoveDownLevel()
{
	if(!movingDownLevel && !movingUpLevel && (currentLevel -1>-1))
	{
		movingDownLevel = true;
		currentLevel = currentLevel -1;
	}
}

bool Lift::MoveDownLevel()
{
	//check if movingDownLevel is true
	if (movingDownLevel)
	{
		//check if our position is greater than the level we want to move to and bottom limit switch is not pressed
		//if ^ is true move lift down if ^ is false stop motor and set movingdownlevel to false
		if ((liftEncoder->GetDistance() > levelEncoValues[currentLevel]) && lowerBound->Get() == RELEASED)
		{
			liftMotor->Set(FULL_SPEED_DOWN);
		}
		else
		{
			liftMotor->Set(0);
			movingDownLevel = false;
		}
	}
	return movingDownLevel;
}

void Lift::CheckLowerBoundLimit()
{
	//Check if the bottom limit switch is pressed. If it is, reset the encoder
	if(lowerBound->Get() == PRESSED)
	{
		liftEncoder->Reset();
	}
}

//Autonomous functions
void Lift::SetUpAuto(AutoInstructions instructions)
{
	//TODO fill this in
}

int Lift::Auto(AutoInstructions instructions)
{
	//TODO fill this in
	return 0;
}
