#include "AutoTimer.h"

AutoTimer::AutoTimer(string name) : MORESubsystem(name)
{
	clock = new Timer();
}

AutoTimer::~AutoTimer()
{
	delete clock;
}

void AutoTimer::SetUpAuto(AutoInstructions instructions)
{
	clock->Reset();
	clock->Start();
}

//param1: time to wait
//param2 - param4: unused
int AutoTimer::Auto(AutoInstructions instructions)
{
	double time = instructions.param1;
	if(clock->Get() < time)
	{
		 return (int)(true);
	}
	else
	{
		return (int)(false);
	}
}
