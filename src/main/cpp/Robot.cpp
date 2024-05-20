// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//test

#include "Robot.h"
#include "math.h"

using namespace std;

void Robot::RobotInit()
{
	intakeShooter.init();
	swerve.init();
	trajectory = trajectoryMaker::MakeTrajectory(frc::filesystem::GetDeployDirectory() + "/test.traj");
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	autonomousTimer.Restart();
	i = 0;
	swerve.resetPos();
}
void Robot::AutonomousPeriodic() {
	while (autonomousTimer.HasElapsed(trajectory[i].timestamp) && i < trajectory.size()) {
		i++;
	}
	if (i == trajectory.size()) {
		swerve.set(complex<double>(0,0), 0);
	} else {
		swerve.SetPose(trajectory[i]);
	}
}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic(){
	double dB = 0.03;
	complex<double> v = complex<double>(-controller.GetLeftY(), -controller.GetLeftX());
	double tR = -controller.GetRightX();
	// apply smooth deadband
	complex<double> velocity = (abs(v)>dB) ? v*(1 - dB/abs(v))/(1-dB) : 0;
	double turn_rate = (abs(tR)>dB) ? tR*(1 - dB/abs(tR))/(1-dB) : 0;
	swerve.set(velocity*constants::max_speed_meters_per_second, turn_rate*constants::max_speed_meters_per_second);
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
