#include "Lift.h"

Lift::Lift(int talonDeviceNumber, int encoAPort, int encoBPort, int upperBoundPort, int lowerBoundPort, string name) : MORESubsystem(name)
{
	liftMotor = new CANTalon(talonDeviceNumber);
	liftEncoder = new Encoder(encoAPort, encoBPort);
	liftEncoder->SetDistancePerPulse(DISTANCE_PER_PULSE);
	liftEncoder->Reset();

	upperBound = new DigitalInput(upperBoundPort);
	lowerBound = new DigitalInput (lowerBoundPort);

	movingToLevel = false;
	integral = 0;

	currentLevel = 0;
	levelEncoValues[0] = 0;
	levelEncoValues[1] = HEIGHT_OF_SCORING_PLAT + (HEIGHT_OF_TOTE) + 3;
	levelEncoValues[2] = HEIGHT_OF_SCORING_PLAT + (2*HEIGHT_OF_TOTE) + 3;
	levelEncoValues[3] = HEIGHT_OF_SCORING_PLAT + (3*HEIGHT_OF_TOTE) + 3;
	levelEncoValues[4] = HEIGHT_OF_SCORING_PLAT + (4*HEIGHT_OF_TOTE) + 3;
	levelEncoValues[5] = HEIGHT_OF_SCORING_PLAT + (5*HEIGHT_OF_TOTE) + 3;
	levelEncoValues[6] = HEIGHT_OF_SCORING_PLAT + (6*HEIGHT_OF_TOTE) + 3;
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

void Lift::StartMoveToLevel(int level)
{
	if(!movingToLevel)
	{
		//Put the level between 0 and 6
		if(level > 6)
			currentLevel = 6;
		else if(level < 0)
			currentLevel = 0;
		else
			currentLevel = level;

		integral = 0;

		movingToLevel = true;
	}
}

bool Lift::MoveToLevel()
{
	float posSetPoint = levelEncoValues[currentLevel];
	float posCurLoc = liftEncoder->GetDistance();//Position current location
	float posError;
	float speedSetPoint;
	float curSpeed = liftEncoder->GetRate();
	float speedError;
	float motorOutput;
	//TODO tune this
	float posKP = 0.01;
	float speedKP = 0.01;
	float speedKI = 0.001;

	//Check if the lift is with 0.25in of the level
	if(abs(posSetPoint - posCurLoc) > 0.25)
	{
		//Calculate the desired speed in in/s
		posError = posSetPoint - posCurLoc;
		speedSetPoint = posError * posKP;

		//Limit the speed
		if(speedSetPoint > SPEED_LIMIT)
		{
			speedSetPoint = SPEED_LIMIT;
		}
		if(speedSetPoint < (-SPEED_LIMIT))
		{
			speedSetPoint = -SPEED_LIMIT;
		}

		//Calculate the motor output [-1, 1]
		speedError = speedSetPoint - curSpeed;
		integral += speedError;
		motorOutput = (speedError * speedKP) + (integral * speedKI);

		//Move the motor yo
		Move(motorOutput);
	}
	else
	{
		//We done
		Move(0);
		movingToLevel = false;
	}

	return movingToLevel;
}

/*void Lift::StartMoveUpLevel()
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
}*/

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
