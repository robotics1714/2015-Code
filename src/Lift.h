#ifndef LIFT_H
#define LIFT_H

#include "CANTalon.h"
#include "Encoder.h"
#include "DigitalInput.h"
#include "MORESubsystem.h"
#include "GlobalDefines.h"

class Lift : public MORESubsystem
{
private:
	CANTalon* liftMotor;
	Encoder*  liftEncoder;
	DigitalInput* upperBound;
	DigitalInput* lowerBound;
	bool movingUpLevel;
	bool movingDownLevel;
	int currentLevel;
	int levelEncoValues[7];

public:
	static const int FULL_SPEED_UP = 1;
	static const int FULL_SPEED_DOWN = -1;

	Lift(int talonDeviceNumber, int encoAPort, int encoBPort, int upperBoundPort, int lowerBoundPort,string name);
	~Lift();

	//Will move the lift up and down
	void Move(float speed);

	bool MoveUpLevel();
	bool MoveDownLevel();

	void StartMoveUpLevel();
	void StartMoveDownLevel();

	//This functions will check if the bottom limit switch is pressed. If it is reset the encoder
	void CheckLowerBoundLimit();

	//MORESubsystem auto functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};

#endif
