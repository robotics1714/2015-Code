#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <string>

using namespace std;

/**
 * A structure that will hold the instructions for performing the autonomous routine for a MORESubsystem
 */
struct AutoInstructions
{
	int flags;
	double param1;
	double param2;
	double param3;
	double param4;
};

/**
 * An abstract class that all subsystems on the robot will inherit
 *
 * Contains the SetUpAuto and Auto functions that all subsystems will override to specify their autonomous actions
 */
class MORESubsystem
{
private:
	string subsystemName;
public:
	/**
	 * The constructor for the MORESubsystem class
	 *
	 * @param name the name of the subsystem
	 */
	MORESubsystem(string name);
	/**
	 * The deconstructor for the MORESubsystem class
	 */
	~MORESubsystem();

	/**
	 * returns the name of the subsystem
	 */
	string getName(){return subsystemName;}
	/**
	 * What children of MORESubsystem will use to set up their autonomous routine
	 *
	 * @param The instructions to perform the autonomous routine
	 */
	virtual void SetUpAuto(AutoInstructions instructions)=0;//Having an =0 at the end of the function declaration makes a function a pure virtual function
	/**
	 * What children of MORESubsystem will use to perform their autonomous routine
	 *
	 * @param instructions The instructions to perform the autonomous routine
	 * @return Returns the status of the subsystem's autonomous routine
	 */
	virtual int Auto(AutoInstructions instructions)=0;
};

#endif
