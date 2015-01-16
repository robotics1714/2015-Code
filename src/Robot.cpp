#include <MORESubsystem.h>
#include "WPILib.h"
#include "AutoStep.h"
#include "DriveTrain.h"

#define FRONT_LEFT_DRIVE_PORT 0
#define REAR_LEFT_DRIVE_PORT 1
#define FRONT_RIGHT_DRIVE_PORT 2
#define REAR_RIGHT_DRIVE_PORT 3

#define LEFT_BUMP_LIMIT_PORT 0
#define RIGHT_BUMP_LIMIT_PORT 1

#define GYRO_PORT 0

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	Joystick* right;
	Joystick* left;
	DriveTrain* drive;
	Gyro* gyro;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		right = new Joystick(0);
		left = new Joystick(1);

		gyro = new Gyro(GYRO_PORT);

		drive = new DriveTrain(FRONT_LEFT_DRIVE_PORT, REAR_LEFT_DRIVE_PORT,
				FRONT_RIGHT_DRIVE_PORT, REAR_RIGHT_DRIVE_PORT, LEFT_BUMP_LIMIT_PORT,
				RIGHT_BUMP_LIMIT_PORT, gyro, "MECANUM");
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
		drive->Drive(right->GetX(), right->GetY(), left->GetX()*-1);
		SmartDashboard::PutNumber("X", left->GetX());
		SmartDashboard::PutNumber("Y", left->GetY()*-1);
		SmartDashboard::PutNumber("Twist", left->GetX());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
