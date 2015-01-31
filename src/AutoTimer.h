#ifndef AUTOTIMER_H
#define AUTOTIMER_H

#include "Timer.h"
#include "MORESubsystem.h"

class AutoTimer : public MORESubsystem
{
private:
	Timer* clock;
public:
	/**
	 * The constructor for the AutoTimer class
	 *
	 * @param name The name of the subsystem.
	 */
	AutoTimer(string name);
	/**
	 * The deconstructor for the AutoTimer class
	 */
	~AutoTimer();

	/**
	 * Will reset and start the clock.
	 *
	 * @param instructions The instructions that the function will use to perform the correct action
	 * 	flags: none
	 * 	param1: time to wait
	 * 	param2 - param4: unused
	 */
	void SetUpAuto(AutoInstructions instructions) override;
	/**
	 * Performs the waiting action.
	 *
	 * @param instructions The instructions that the function will use to perform the correct action
	 * 	flags: none
	 * 	param1: time to wait
	 * 	param2 - param4: unused
	 *
	 * @return Returns 1 if the time waited is less than the time param1 specified to wait. Returns 0 otherwise
	 */
	int Auto(AutoInstructions instructions) override;
};


#endif
