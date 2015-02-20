#include "Lift.h"

Lift::Lift(int talonDeviceNumber, int liftPotPort, int encoAPort, int encoBPort,
		int upperBoundPort, int lowerBoundPort, Spatula* spatula, string robotNumber, string name) : MORESubsystem(name)
{
	liftMotor = new CANTalon(talonDeviceNumber);
	liftEncoder = new Encoder(encoAPort, encoBPort);
	liftEncoder->SetDistancePerPulse(DISTANCE_PER_PULSE);
	liftEncoder->Reset();

	liftPot = new AnalogInput(liftPotPort);

	upperBound = new DigitalInput(upperBoundPort);
	lowerBound = new DigitalInput (lowerBoundPort);

	spat = spatula;

	movingToLevel = false;
	integral = 0;

	manuallyMoving = false;

	acquireState = IDLE_STATE;

	//Use these values for the practice bot
	if(robotNumber == "2")
	{
		currentLevel = 0;
		levelPotValues[0] = 860;
		levelPotValues[1] = 2000;
		levelPotValues[2] = 2500;
		levelPotValues[3] = 3050;
		levelPotValues[4] = 3520;
		levelPotValues[5] = 3550;
	}
	//Use these values for the competition bot
	else
	{
		currentLevel = 0;
		levelPotValues[0] = 860;
		levelPotValues[1] = 2000;
		levelPotValues[2] = 2500;
		levelPotValues[3] = 3050;
		levelPotValues[4] = 3520;
		levelPotValues[5] = 3550;
	}
}

Lift::~Lift()
{
	delete liftMotor;
	delete liftEncoder;
	delete upperBound;
	delete lowerBound;
	delete spat;
}

bool Lift::Move(float speed)
{
	//Make sure we stop when we hit a limit switch
	if( ((speed < 0) && (upperBound->Get() == RELEASED) && (liftPot->GetAverageValue() < levelPotValues[5]))
			|| ((speed > 0) && lowerBound->Get() == RELEASED) )
	{
		liftMotor->Set(speed);
		return true;
	}
	else
	{
		liftMotor->Set(0);
		return false;
	}
}

void Lift::StartMoveToLevel(int level)
{
	if(!movingToLevel)
	{
		//Put the level between 0 and 6
		if(level > 5)
			currentLevel = 5;
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
	float posSetPoint = levelPotValues[currentLevel];
	float posCurLoc = liftPot->GetAverageValue();//Position current location
	float posError;
	float speedSetPoint;
	float curSpeed = liftEncoder->GetRate();
	float speedError;
	float motorOutput;
	//TODO tune this
	float posKP = 0.02;
	float speedKP = 0.07;
	float speedKI = 0.0001;
	ofstream file;

	//Check if the lift is with 0.25in of the level
	if((fabs(posSetPoint - posCurLoc) > 50) && (movingToLevel))
	{
		//Calculate the desired speed in in/s
		posError = posCurLoc - posSetPoint;
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
		integral += (speedError * speedKI);
		motorOutput = (speedError * speedKP) + (integral);

		//If the motor output is 100%, we don't need to add to the integral, so get rid of the integral we added this loop
		if((motorOutput >= 1) || (motorOutput <= -1))
		{
			integral -= (speedError * speedKI);
		}

		//Move the motor yo
		//Check if this is false, meaning a limit switch was pressed, so stop it
		if(!Move(motorOutput))
		{
			Move(0);
			SmartDashboard::PutString("HERE", "HERE");
			movingToLevel = false;
		}

		SmartDashboard::PutNumber("Pos Error", posError);
		SmartDashboard::PutNumber("Speed Error", speedError);

		//Save the position P loop information into a file for easy tuning
		file.open("/home/lvuser/position1.csv");
		//Set point, controller output, actual value
		file<<posSetPoint<<","<<speedSetPoint<<","<<posCurLoc<<endl;
		file.close();

		//Save the speed PI loop information into a file for easy tuning
		file.open("/home/lvuser/speed1.csv");
		//Set point, controller output, actual value
		file<<speedSetPoint<<","<<motorOutput<<","<<curSpeed<<endl;
		file.close();
	}
	else if(!manuallyMoving)
	{
		//We done
		Move(0);
		movingToLevel = false;
	}

	//Reset manuallyMoving for the next loop
	manuallyMoving = false;

	SmartDashboard::PutNumber("speedSetPoint", speedSetPoint);
	SmartDashboard::PutNumber("motorOutput", motorOutput);

	return movingToLevel;
}

void Lift::StartAcquire()
{
	if(acquireState == IDLE_STATE && !movingToLevel)
	{
		//Set the state to the grab state and start grabbin'
		acquireState = ACQUIRE_GRAB_STATE;
		spat->StartMoveUp();
	}
}

int Lift::Acquire()
{
	switch(acquireState)
	{
	case ACQUIRE_GRAB_STATE:
		//Move down the spatula, when it's all the way down, start raising the lift to level 1
		if(!spat->MoveUp())
		{
			acquireState = ACQUIRE_LIFT_STATE;
			StartMoveToLevel(1);
		}
		break;
	case ACQUIRE_LIFT_STATE:
		//Move the lift to level 1, when it gets there, we done
		if(!movingToLevel)
		{
			acquireState = IDLE_STATE;
		}
		break;
	}

	return acquireState;
}

void Lift::Stop()
{
	//Stop the motor
	liftMotor->Set(0);
	spat->Stop();
	//signal that the lift is not moving to level or acquiring
	movingToLevel = false;
	acquireState = IDLE_STATE;
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

/*void Lift::CheckLowerBoundLimit()
{
	//Check if the bottom limit switch is pressed. If it is, reset the encoder
	if(lowerBound->Get() == PRESSED)
	{
		liftEncoder->Reset();
	}
}*/

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
