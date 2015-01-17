#ifndef AUTOTIMER_H
#define AUTOTIMER_H

#include "Timer.h"
#include "MORESubsystem.h"

class AutoTimer : public MORESubsystem
{
private:
	Timer* clock;
public:
	AutoTimer(string name);
	~AutoTimer();

	void SetUpAuto(AutoInstructions instructions);
	int Auto(AutoInstructions instructions);
};


#endif
