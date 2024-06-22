// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//test

#include "Robot.h"

using namespace std;

void Robot::RobotInit()
{
	intakeShooter.Init();
	swerve.Init();
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	swerve.AutonomousInit();
	swerve.SetTrajectory(trajectory);
}
void Robot::AutonomousPeriodic() {
	swerve.FollowTrajectory();
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){
	swerve.SetVelocity(controller.GetRawAxis(0), controller.GetRawAxis(1), controller.GetRawAxis(2));
	if (controller.GetRawButton(4)) {
		swerve.ResetAngle();
	}
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
	return frc::StartRobot<Robot>();
}
#endif
