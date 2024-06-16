// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>
#include <utils/mathFunctions.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DutyCycleEncoder.h>
#include <string>

class IntakeShooter {
public:
	frc::DigitalInput eye0{0};
	frc::DigitalInput eye1{1};
    frc::DigitalInput eye2{2};
 	void SetAngle(double angle) {
		this->angleSetpoint = angle;
 	}
	void SetP(double P = 0.0125) {
		this->P = P;
	}
	void SetI(double I = 0) {
		this->I = I;
	}
	void SetD(double D = 0) {
		this->D = D;
	}
	void SetF(double F = 0) {
		this->F = F;
	}
	void SetOutputRange(double min = -0.25, double max = 0.3) {
		this->max = max;
		this->min = min;
	}
 	void RunAnglePID() {
		double currentAngle = GetAngle();
		angleError = angleSetpoint - currentAngle;
		mf::wrapDeg(angleError);
		double output = angleError*P;
		accumulator += angleError*I;
		output += accumulator;
		double angleChange = currentAngle - lastAngle;
		mf::wrapDeg(angleChange);
		output -= angleChange*D;
		output += F;
		if (output > max) {
			output = max;
		}
		if (output < min) {
			output = min;
		}
		m_angle.Set(output);
 	}
	bool GetAngleReached(double tolerance = 2) {
		return abs(angleError) < tolerance;
	}
	void SetIntakeSpeed(double inPerSec) {
		intakePID.SetReference(inPerSec*60, rev::CANSparkMax::ControlType::kVelocity);
	}
    void SetIntake(double percent){
        m_intake.Set(percent/100);
    }
	void SetShooterSpeed(double inPerSec) { 
		/* Use torque velocity */
		m1_shooter.SetControl(s_velocity.WithVelocity(inPerSec/shooterIPR*1_tps).WithFeedForward(20_A));
		m2_shooter.SetControl(s_velocity.WithVelocity(inPerSec/shooterIPR*1_tps).WithFeedForward(15_A));
	}
	void SetShooter(double speed) {
		m1_shooter.Set(speed/100);
		m2_shooter.Set(speed/100);
	}
	int GetNotePresent() {
		return eye0.Get() || eye1.Get() || eye2.Get();
	}
	double GetAngle() {
		return -e_abs_angle.GetDistance();
	}
 	void Init() {
		m_intake.RestoreFactoryDefaults();
        m_intake.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
		m_intake.SetInverted(true);
		intakePID.SetP(6e-5);
		intakePID.SetI(0);
		intakePID.SetD(0);
		intakePID.SetFF(0.000015);
		m_intake.BurnFlash();
		
 		m_angle.RestoreFactoryDefaults();
        m_angle.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
 		m_angle.SetInverted(false);

		e_abs_angle.SetDistancePerRotation(360.0);
		e_abs_angle.SetPositionOffset(351);
 		m_angle.BurnFlash();
		lastAngle = GetAngle();
 	}
	
private:
	double angleSetpoint = 15;
	double angleError;
	double lastAngle;
	double accumulator = 0;
	double P = 0.008;
	double I = 0;
	double D = 0;
	double F = -0.015;
	double max = 0.25;
	double min = -0.25;
	rev::CANSparkMax m_intake{42, rev::CANSparkMax::MotorType::kBrushless};
	rev::SparkPIDController intakePID = m_intake.GetPIDController();
	
	ctre::phoenix6::controls::VelocityTorqueCurrentFOC s_velocity{0_tps, 0_tr_per_s_sq, 0_A, 1, false};
	ctre::phoenix6::hardware::TalonFX m1_shooter{43, "CTREdevices"};
	ctre::phoenix6::hardware::TalonFX m2_shooter{44, "CTREdevices"};

 	rev::CANSparkMax m_angle{41, rev::CANSparkMax::MotorType::kBrushless};
 	// rev::SparkPIDController anglePID = m_angle.GetPIDController();
 	// rev::SparkRelativeEncoder e_angle = m_angle.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);

	frc::DutyCycleEncoder e_abs_angle{9};
	
	const double intakeGearboxReduction = 9;
	// inches per rotation of the intake motor
	const double intakeIPR = M_PI*2/intakeGearboxReduction;
	// inches per rotation of the shooter motors
	const double shooterIPR = M_PI*4;

const double angleGearboxReduction = 60;
// degrees per rotation of the angle motor
const double angleDPR = 360/angleGearboxReduction;
} intakeShooter;
