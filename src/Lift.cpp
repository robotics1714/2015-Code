#include "Lift.h"

Lift::Lift(int talonDeviceNumber, string name) : MORESubsystem(name)
{
	liftMotor = new CANTalon(talonDeviceNumber);
}

Lift::~Lift()
{
	delete liftMotor;
}

void Lift::Move(float speed)
{
	liftMotor->Set(speed);
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
