#ifndef AUTOSTEP_H
#define AUTOSTEP_H

#include <vector>
#include "MORESubsystem.h"

using namespace std;

//This class will be in charge of keeping track of one step of the autonomous routine
class AutoStep
{
private:
	//We need vectors to hold all of the subsystems and instructions to be used in a step 
	vector<MORESubsystem*> subsystems;
	vector<AutoInstructions> instructions;
	
	bool firstLoop;//This variable keeps track of if it is the first time we are looping through this step
	
public:
	AutoStep(MORESubsystem* subsystem, AutoInstructions instructions);//This class will require at least 1 set of subsystems and instructions
	~AutoStep();
	
	//For when the auton routine requires two things to be happening at once
	void AddSubsystem(MORESubsystem* subsystem, AutoInstructions instructions);
	
	//This function will be the one that actually performs the autonomous step
	//It will return true if it is still performing and false when it is not
	bool PerformStep();
};

#endif
