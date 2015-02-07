#ifndef LIFT_H
#define LIFT_H

#include <cmath>
#include <fstream>
#include "SmartDashboard/SmartDashboard.h"
#include "CANTalon.h"
#include "Victor.h"
#include "Encoder.h"
#include "DigitalInput.h"
#include "AnalogInput.h"
#include "MORESubsystem.h"
#include "GlobalDefines.h"
#include "Spatula.h"

#define DISTANCE_PER_PULSE 0.01 //TODO calculate this, it should be (distance the lift travels)/(encoder pulses to travel entire lift)
#define HEIGHT_OF_TOTE 400//Pot values that relate to the height of the platform/totes
#define HEIGHT_OF_SCORING_PLAT 150

#define SPEED_LIMIT 2//3 //the max speed the MoveToLevel functions can make the lift go in in/s

class Lift : public MORESubsystem
{
private:
	CANTalon* liftMotor;
	Encoder*  liftEncoder;
	DigitalInput* upperBound;
	DigitalInput* lowerBound;
	AnalogInput* liftPot;
	Spatula* spat;
	//bool movingUpLevel;
	//bool movingDownLevel;
	bool movingToLevel;
	int currentLevel;
	int levelPotValues[7];
	float integral;
	int acquireState;
public:
	static const int FULL_SPEED_UP = -1;/**< Constant for moving the lift up at full speed*/
	static const int FULL_SPEED_DOWN = 1;/**< Constant for moving the lift down at full speed*/

	static const int ACQUIRE_GRAB_STATE = 1;/**< Constant for the value of the grab state for the acquire state machine*/
	static const int ACQUIRE_LIFT_STATE = 2;/**< Constant for the value of the lift state for the acquire state machine*/

	/**
	 * The constructor for the Lift class
	 *
	 * @param talonDeviceNumber The CAN device id number for the TalonSRX that will drive the lift
	 * @param liftPotPort The port for the potentiometer on the Lift
	 * @param encoAPort The port for the A channel of the encoder on the Lift
	 * @param encoBPort The port for the B channel of the encoder on the Lift
	 * @param upperBoundPort The port for the limit switch that will be used as the upper bound of the lift
	 * @param lowerBoundPort The port for the limit switch that will be used as the lower bound of the lift
	 * @param name the name of the subsystem
	 */
	Lift(int talonDeviceNumber, int liftPotPort, int encoAPort, int encoBPort,
			int upperBoundPort, int lowerBoundPort, string name);
	/**
	 * The deconstructor for the Lift class
	 */
	~Lift();

	/**
	 * Moves the lift up or down.
	 *
	 * Moves the lift up or down until either the upper bound or lower bound limit switches are pressed.
	 *
	 * @param speed The speed and direction to move the lift
	 * @return Returns true if the lift is allowed to move. Returns false if the lift hits one of the two limit switches
	 */
	bool Move(float speed);

	//bool MoveUpLevel();
	//bool MoveDownLevel();

	//void StartMoveUpLevel();
	//void StartMoveDownLevel();

	/**
	 * Moves the lift to the level specified in the StartMoveToLevel function
	 *
	 * Moves the lift to the specified level and stops when it is within 0.5 inches away from the ideal location.
	 * This function should be called in every call of TeleopPeriodic
	 *
	 * @see StartMoveToLevel(int level)
	 * @return Returns true if the lift is in the process of moving. Returns false when it has reached the level
	 */
	bool MoveToLevel();
	/**
	 * Function that will allow the MoveToLevel() function start
	 *
	 * @param level the level [0-6] to move the motor to
	 * @see MoveToLevel()
	 */
	void StartMoveToLevel(int level);

	/**
	 * Functions that will go through the steps of acquiring a tote/container
	 *
	 * When in AQUIRE_GRAB_STATE it will move the spatula into grabbing position
	 * When in AQUIRE_LIFT_STATE the lift will go to level 1
	 *
	 * @return Returns the current state
	 */
	int Acquire();
	/**
	 * Function that will allow the Acquire() function to start.
	 *
	 * Checks to see if the lift's acquire state is idle and if it is sets the state to AQUIRE_GRAB_STATE
	 *
	 * @see Acquire()
	 */
	void StartAcquire();

	/**
	 * stops all action on the lift
	 */
	void Stop();

	//This functions will check if the bottom limit switch is pressed. If it is reset the encoder
	//void CheckLowerBoundLimit();

	/**
	 * @return An instance of the lift encoder
	 */
	Encoder* GetEnco(){return liftEncoder;}
	/**
	 * @return An instance of the lift potentiometer
	 */
	AnalogInput* GetPot(){return liftPot;}

	/**
	 * @return An instance of the upper bound limit switch
	 */
	DigitalInput* getUpperBound(){return upperBound;}

	/**
	 * @return An instance of the lower bound limit switch
	 */
	DigitalInput* getLowerBound(){return lowerBound;}

	/**
	 * @return Returns whether or not the lift is moving to a level
	 */
	bool GetMovingToLevel(){return movingToLevel;}

	//MORESubsystem auto functions
	void SetUpAuto(AutoInstructions instructions) override;
	int Auto(AutoInstructions instructions) override;
};

#endif
