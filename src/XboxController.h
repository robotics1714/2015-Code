
/*
 * File: XboxController.h
 * Description: h file for the XboxController
 * Last Changed: 1-19-13 borderline 1-20-13 :D
 * Author: Matthew W
 */

#ifndef XBOXCONTROLLER_H
#define XBOXCONTROLLER_H

#include <Joystick.h>

class XboxController
{

private:
	Joystick* xboxController;

public:
	/**
	 * Constructor for the XboxController
	 *
	 * @param parXboxController An instance of a joystick that represents the XBox controller
	 */
	XboxController(Joystick* parXboxController);
	/**
	 * Deconstructor for the XboxController
	 */
	~XboxController();

	/*
	 * Use these methods for easier methods on reading input
	 * from an xbox controller
	 */

	/**
	 * @return Returns true if A is pressed. Otherwise, return false
	 */
	bool IsAPressed();
	/**
	 * @return Returns true if B is pressed. Otherwise, return false
	 */
	bool IsBPressed();
	/**
	 * @return Returns true if X is pressed. Otherwise, return false
	 */
	bool IsXPressed();
	/**
	 * @return Returns true if Y is pressed. Otherwise, return false
	 */
	bool IsYPressed();
	/**
	 * @return Returns true if the left bumper is pressed. Otherwise, return false
	 */
	bool IsLeftBumperPressed();
	/**
	 * @return Returns true if right bumper is pressed. Otherwise, return false
	 */
	bool IsRightBumperPressed();
	/**
	 * @return Returns true if the back button is pressed. Otherwise, return false
	 */
	bool IsBackButtonPressed();
	/**
	 * @return Returns true if the start button is pressed. Otherwise, return false
	 */
	bool IsStartButtonPressed();
	/**
	 * @return Returns true if the left stick is pressed. Otherwise, return false
	 */
	bool IsLeftStickPressed();
	/**
	 * @return Returns true if the right stick is pressed. Otherwise, return false
	 */
	bool IsRightStickPressed();

	/*
	 * LeftXAxis: Finds the x axis of the left joysick on the xbox controller
	 * LeftYAxis: Finds the y axis of the left joystick
	 * TriggerAxis: Finds the axis of both the triggers combined, with the left trigger making the value more negative and the right trigger making the value more positive
	 * RightXAxis: Finds the x axis of the right joystick
	 * RightYAxis: Finds the y axis of the right joystick
	 */

	/**
	 * @return Returns the left X axis value of the controller
	 */
	double GetLeftXAxis();
	/**
	 * @return Returns the left Y axis value of the controller
	 */
	double GetLeftYAxis();
	/**
	 * @return Returns the left trigger axis value of the controller
	 */
	double GetLeftTriggerAxis();
	/**
	 * @return Returns the right trigger axis value of the controller
	 */
	double GetRightTriggerAxis();
	/**
	 * @return Returns the right X axis value of the controller
	 */
	double GetRightXAxis();
	/**
	 * @return Returns the right Y axis value of the controller
	 */
	double GetRightYAxis();

};

#endif
