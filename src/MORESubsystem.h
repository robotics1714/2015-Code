#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

#include <string>

using namespace std;

struct AutoInstructions
{
	int flags;
	double param1;
	double param2;
	double param3;
};

class MORESubsystem
{
private:
	string subsystemName;
public:
	MORESubsystem(string name);
	~MORESubsystem();

	string getName(){return subsystemName;}
	virtual void SetUpAuto(AutoInstructions instructions)=0;//Having an =0 at the end of the function declaration makes a function a pure virtual function
	virtual int Auto(AutoInstructions instructions)=0;
};

#endif
