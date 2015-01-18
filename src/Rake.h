/*
 * Rake.h
 *
 *  Created on: Jan 17, 2015
 *      Author: general.user
 */

#ifndef RAKE_H
#define RAKE_H

#include "MORESubsystem.h"
#include "CANTalon.h"
#include "Timer.h"

class Rake : public MORESubsystem
{
private:
	CANTalon* winch;
	Timer* moveTimer;
	float moveTimeSpeed;
	float moveTimeDuration;
	bool movingForTime;
public:
	Rake(int talonDeviceNumber, string name);
	~Rake();

	///Speed of the winch
	void Move(float speed);

	///Starts the movement timer
	void StartMoveForTime(float speed, float time);

	bool MoveForTime();
	///Stop Motor
	void Stop();


	///MORESubsystem Auto Instructions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};


#endif /* SRC_RAKE_H_ */