#ifndef AUTOSTEP_H
#define AUTOSTEP_H

#include <vector>
#include "MORESubsystem.h"

using namespace std;

/**
 * This class keeps track of one step of an autonomous routine.
 *
 * The AutoStep class will take in instances of MORESubsystem and AutoInstructions and call the subsystem's Auto function.
 */
class AutoStep
{
private:
	//We need vectors to hold all of the subsystems and instructions to be used in a step 
	vector<MORESubsystem*> subsystems;
	vector<AutoInstructions> instructions;
	string stepName;
	
	bool firstLoop;//This variable keeps track of if it is the first time we are looping through this step
	
public:
	/**
	 * The constructor for the AutoStep class.
	 *
	 * @param subsystem An instance of MORESubsystem whose Auto() function is called to perform an action in autonomous.
	 * @param instructions The instructions the Auto() function of the included MORESubstance will use to perform the correct autonomous action.
	 * @param name The name of the AutoStep. The main purpose for the name parameter is debugging
	 */
	AutoStep(MORESubsystem* subsystem, AutoInstructions instructions, string name);//This class will require at least 1 set of subsystems and instructions
	/**
	 * The deconstructor for the AutoStep class.
	 */
	~AutoStep();
	
	//For when the auto routine requires two things to be happening at once
	/**
	 * Adds another MORESubsystem and set of AutoInstructions to the AutoStep.
	 *
	 * This is used when a step in autonomous needs multiple subsystems to do things at once
	 *
	 * @param subsystem An instance of MORESubsystem whose Auto() function is called to perform in action in this autonomous step.
	 * @param instructions The instructions the Auto() function of the included MORESubsystem will use to perform the correct autonomous action.
	 */
	void AddSubsystem(MORESubsystem* subsystem, AutoInstructions instructions);
	
	/**
	 * Returns the name of the step
	 *
	 * @return the name of the step
	 */
	string GetStepName(){return stepName;}

	/*
	 * This function will be the one that actually performs the autonomous step.
	 *
	 * It calls the Auto() of every MORESubsystem in this step.
	 *
	 * @return This function will return true if the step is still being performed. If the step has finished, the function will return false.
	 */
	bool PerformStep();
};

#endif
