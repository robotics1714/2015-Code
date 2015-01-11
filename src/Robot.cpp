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
	Joystick* joy;
	DriveTrain* drive;
	Gyro* gyro;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();

		joy = new Joystick(0);

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
		drive->Drive(joy->GetX(), joy->GetY(), joy->GetTwist());
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
