#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/smartdashboard/SmartDashboard.h"
#include <rev/CANSparkMax.h>
#include <string>
#include "utils/mathFunctions.h"
#include "constants.h"

using namespace std;
using namespace ctre::phoenix6;
using namespace constants;

class SwerveModule{
public:
    SwerveModule(int modID, double position_x, double position_y){
        complex<double> position = complex<double>(position_x, position_y);
        this->modID = modID;
        turn_vector = position*complex<double>(0, 1)/abs(position);
        m_drive = new hardware::TalonFX(modID+10, "CTREdevices");
        m_steering = new hardware::TalonFX(modID+20, "CTREdevices");
        encoder = new hardware::CANcoder(modID+30, "CTREdevices");
    }

    void init() {
        m_drive->SetNeutralMode(signals::NeutralModeValue::Brake);
        configs::TalonFXConfiguration configs{};
		/* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		configs.Slot0.kP = 5; // An error of 1 rotation per second results in 5 amps output
		configs.Slot0.kS = feedforward_current;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = max_current;
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -max_current;
        m_drive->GetConfigurator().Apply(configs, 50_ms);
        m_steering->GetConfigurator().Apply(configs, 50_ms);
        m_drive->GetPosition().SetUpdateFrequency(100_Hz);
        m_drive->GetVelocity().SetUpdateFrequency(100_Hz);
        encoder->GetAbsolutePosition().SetUpdateFrequency(100_Hz);
    }

    void SetVelocity(complex<double> robot_velocity, double angular_vel, complex<double> robot_accel = complex<double>(0,0), double angular_accel = 0){
        complex<double> velocity = FindModuleVector(robot_velocity, angular_vel);
        complex<double> accel_current = FindModuleVector(robot_accel, angular_accel)*current_to_accel_ratio;
        double wheel_speed = abs(velocity);
        angle = encoder->GetAbsolutePosition().GetValueAsDouble()*tau;
        double error = arg(velocity) - angle;
        mf::wrap(error);
        if (wheel_speed < 0.008) {
            error = 0;
        }
        if (abs(error) > M_PI/2){
            error += M_PI;
            mf::wrap(error);
            wheel_speed *= -1;
        }
        m_steering->Set(error/M_PI);
        /* Use torque velocity */
        double wheel_accel_current = mf::GetProjectionMagnitude(accel_current, polar<double>(1, angle));
        m_drive->SetControl(velocity_ctrl.WithVelocity(wheel_speed*motor_turns_per_m * 1_tps).WithFeedForward(wheel_accel_current*1_A));
        frc::SmartDashboard::PutNumber("setpoint speed" + to_string(modID), abs(wheel_speed*motor_turns_per_m));
        frc::SmartDashboard::PutNumber("actual speed" + to_string(modID), GetSpeed());
        frc::SmartDashboard::PutNumber("current in amps" + to_string(modID), abs(m_drive->GetClosedLoopOutput().GetValueAsDouble()));
    }

    complex<double> FindModuleVector(complex<double> robot_vector, double angular_rate) {
        return robot_vector + turn_vector * angular_rate;
    }

    double GetAccelOvershoot(complex<double> robot_vel, double angular_vel, complex<double> robot_vel_increment, double angular_vel_increment) {
        complex<double> velocity = FindModuleVector(robot_vel, angular_vel);
        // find velocity increment
        complex<double> vel_increment = FindModuleVector(robot_vel_increment, angular_vel_increment);
        double accel_overshoot = 1;
        if (abs(vel_increment) > max_m_per_sec_per_cycle) {
            accel_overshoot = abs(vel_increment) / max_m_per_sec_per_cycle;
        }
        double wheel_current = mf::GetProjectionMagnitude(vel_increment/cycle_time.value()*current_to_accel_ratio, velocity) + feedforward_current;
        double wheel_accel_overshoot = abs(wheel_current) / (max_current-current_headroom);
        if (wheel_accel_overshoot > accel_overshoot) {
            accel_overshoot = wheel_accel_overshoot;
        }
        return accel_overshoot;
    }

    complex<double> GetPositionChange() {
        angle = encoder->GetAbsolutePosition().GetValueAsDouble()*tau;
        double motor_position = m_drive->GetPosition().GetValueAsDouble();
        double motor_position_change = motor_position - motor_position_old;
        motor_position_old = motor_position;
        complex<double> position_change = polar<double>(motor_position_change / motor_turns_per_m, angle);
        return position_change;
    }

    double GetSpeed() {
        return abs(m_drive->GetVelocity().GetValueAsDouble());
    }

    void resetEncoders() {
        motor_position_old = 0;
        m_drive->SetPosition(0_tr);
    }

private:
    hardware::TalonFX *m_drive;
    hardware::TalonFX *m_steering;
    hardware::CANcoder *encoder;
	controls::VelocityTorqueCurrentFOC velocity_ctrl = controls::VelocityTorqueCurrentFOC{0_tps, 0_tr_per_s_sq, 0_A, 0, false}.WithSlot(0);
    int modID;
    complex<double> turn_vector;
    double angle;
    double motor_position_old = 0;
};
/*

# # # # # # # # # # # # # # # # # # #
# C:11                         C:13 #
# E:21                         E:23 #
# N:31                         N:33 #
#                                   #
#                ^                  #
#                | x                #
#          y <-- +                  #
#                                   #
#                                   #
#                                   #
# C:12                         C:14 #
# E:22                         E:24 #
# N:32                         N:34 #
# # # # # # # # # # # # # # # # # # #

*/
