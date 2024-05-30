#pragma once

#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <frc/Filesystem.h>

#include <fstream>
#include <string.h>
#include "cameraserver/CameraServer.h"

#include "subsystems/Swerve.h"
#include "subsystems/IntakeShooter.h"
#include "subsystems/Limelight.h"
using namespace std;


class Robot : public frc::TimedRobot{
public:
	Robot() : frc::TimedRobot(constants::cycle_time) {
		vector<SwerveModule> modules;
		modules.push_back(SwerveModule{1,  1, 1});
		modules.push_back(SwerveModule{2, -1, 1});
		modules.push_back(SwerveModule{3, -1,-1});
		modules.push_back(SwerveModule{4,  1,-1});
		swerve.AddModules(modules);
	}

	void RobotInit() override;
	void RobotPeriodic() override;

	void AutonomousInit() override;
	void AutonomousPeriodic() override;

	void TeleopInit() override;
	void TeleopPeriodic() override;

	void DisabledInit() override;
	void DisabledPeriodic() override;

	void TestInit() override;
	void TestPeriodic() override;

	void SimulationInit() override;
	void SimulationPeriodic() override;

private:
	Trajectory trajectory{frc::filesystem::GetDeployDirectory() + "/test.traj"};
	Swerve swerve;
	frc::Joystick controller{0};
};