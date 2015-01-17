#ifndef LIFT_H
#define LIFT_H

#include "CANTalon.h"
#include "MORESubsystem.h"

class Lift : public MORESubsystem
{
private:
	CANTalon* liftMotor;

public:
	Lift(int talonDeviceNumber, string name);
	~Lift();

	//Will move the lift up and down
	void Move(float speed);

	//MORESubsystem auto functions
	void SetUpAuto(AutoInstructions instructions);
	int Auto(AutoInstructions instructions);
};

#endif
