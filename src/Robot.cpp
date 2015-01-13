#include <MORESubsystem.h>
#include "WPILib.h"
#include "AutoStep.h"
#include "DriveTrain.h"

#define FRONT_LEFT_DRIVE_PORT 0
#define REAR_LEFT_DRIVE_PORT 1
#define FRONT_RIGHT_DRIVE_PORT 2
#define REAR_RIGHT_DRIVE_PORT 3

#define GYRO_PORT 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Joystick* left;
	Joystick* right;
	DriveTrain* drive;
	Gyro* gyro;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		left = new Joystick(0);
		right = new Joystick(1);

		gyro = new Gyro(GYRO_PORT);

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, gyro, "MECANUM");
	}

	void AutonomousInit()
	{
	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{
		gyro->Reset();
	}

	void TeleopPeriodic()
	{
		drive->Drive(right->GetX()*-1, right->GetY()*-1, left->GetX()*-1);
		SmartDashboard::PutNumber("X", right->GetX());
		SmartDashboard::PutNumber("Y", right->GetY()*-1);
		SmartDashboard::PutNumber("Twist", left->GetX());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
