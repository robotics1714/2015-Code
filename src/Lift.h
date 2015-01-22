#ifndef LIFT_H
#define LIFT_H

#include <cmath>
#include "SmartDashboard/SmartDashboard.h"
#include "CANTalon.h"
#include "Victor.h"
#include "Encoder.h"
#include "DigitalInput.h"
#include "MORESubsystem.h"
#include "GlobalDefines.h"

#define DISTANCE_PER_PULSE 0.01 //TODO calculate this, it should be (distance the lift travels)/(encoder pulses to travel entire lift)
#define HEIGHT_OF_TOTE 2//12.1
#define HEIGHT_OF_SCORING_PLAT 2

#define SPEED_LIMIT 2//3 //the max speed the MoveToLevel functions can make the lift go in in/s

class Lift : public MORESubsystem
{
private:
	Victor* liftMotor;//TODO this should be a CANTalon
	Encoder*  liftEncoder;
	DigitalInput* upperBound;
	DigitalInput* lowerBound;
	//bool movingUpLevel;
	//bool movingDownLevel;
	bool movingToLevel;
	int currentLevel;
	int levelEncoValues[7];
	float integral;
public:
	static const int FULL_SPEED_UP = 1;
	static const int FULL_SPEED_DOWN = -1;

	Lift(int talonDeviceNumber, int encoAPort, int encoBPort, int upperBoundPort, int lowerBoundPort, string name);
	~Lift();

	//Will move the lift up and down
	bool Move(float speed);

	//bool MoveUpLevel();
	//bool MoveDownLevel();

	//void StartMoveUpLevel();
	//void StartMoveDownLevel();

	bool MoveToLevel();
	void StartMoveToLevel(int level);

	//This functions will check if the bottom limit switch is pressed. If it is reset the encoder
	void CheckLowerBoundLimit();

	Encoder* GetEnco(){return liftEncoder;}

	//MORESubsystem auto functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};

#endif
