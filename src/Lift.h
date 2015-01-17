#ifndef LIFT_H
#define LIFT_H

#include "CANTalon.h"
#include "MORESubsystem.h"

class Lift : public MORESubsystem
{
private:
	CANTalon* liftMotor;

public:
	static const int FULL_SPEED_UP = 1;
	static const int FULL_SPEED_DOWN = -1;

	Lift(int talonDeviceNumber, string name);
	~Lift();

	//Will move the lift up and down
	void Move(float speed);

	//MORESubsystem auto functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};

#endif
