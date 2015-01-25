#ifndef LIFT_H
#define LIFT_H

#include <cmath>
#include "SmartDashboard/SmartDashboard.h"
#include "CANTalon.h"
#include "Victor.h"
#include "Encoder.h"
#include "DigitalInput.h"
#include "AnalogInput.h"
#include "MORESubsystem.h"
#include "GlobalDefines.h"
#include "Spatula.h"

#define DISTANCE_PER_PULSE 0.01 //TODO calculate this, it should be (distance the lift travels)/(encoder pulses to travel entire lift)
#define HEIGHT_OF_TOTE 400//Pot values that relate to the height of the platform/totes
#define HEIGHT_OF_SCORING_PLAT 150

#define SPEED_LIMIT 2//3 //the max speed the MoveToLevel functions can make the lift go in in/s

class Lift : public MORESubsystem
{
private:
	CANTalon* liftMotor;//TODO this should be a CANTalon
	Encoder*  liftEncoder;
	DigitalInput* upperBound;
	DigitalInput* lowerBound;
	AnalogInput* liftPot;
	Spatula* spat;
	//bool movingUpLevel;
	//bool movingDownLevel;
	bool movingToLevel;
	int currentLevel;
	int levelPotValues[7];
	float integral;
	int acquireState;
public:
	static const int FULL_SPEED_UP = -1;
	static const int FULL_SPEED_DOWN = 1;

	static const int ACQUIRE_GRAB_STATE = 1;
	static const int ACQUIRE_LIFT_STATE = 2;

	Lift(int talonDeviceNumber, int liftPotPort, int encoAPort, int encoBPort,
			int upperBoundPort, int lowerBoundPort, string name);
	~Lift();

	//Will move the lift up and down
	bool Move(float speed);

	//bool MoveUpLevel();
	//bool MoveDownLevel();

	//void StartMoveUpLevel();
	//void StartMoveDownLevel();

	bool MoveToLevel();
	void StartMoveToLevel(int level);

	int Acquire();
	void StartAcquire();

	//This functions will check if the bottom limit switch is pressed. If it is reset the encoder
	//void CheckLowerBoundLimit();

	Encoder* GetEnco(){return liftEncoder;}
	AnalogInput* GetPot(){return liftPot;}

	//MORESubsystem auto functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};

#endif
