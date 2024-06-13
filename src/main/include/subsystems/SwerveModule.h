#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include "frc/smartdashboard/SmartDashboard.h"
#include <rev/CANSparkMax.h>
#include <complex.h>
#include <string>
#include "utils/angleMath.h"
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
        configs::Slot0Configs& slot0Configs = configs.Slot0;
        configs::Slot1Configs& slot1Configs = configs.Slot1;
		/* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		slot0Configs.kP = 5; // An error of 1 rotation per second results in 5 amps output
		//slot0Configs.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
		slot0Configs.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
        slot1Configs.kP = 10; // An error of 1 rotations results in 40 A output
        slot1Configs.kD = 2; // A velocity of 1 rps results in 2 A output
        configs.TorqueCurrent.PeakForwardTorqueCurrent = max_current.value();
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -max_current.value();
        m_drive->GetConfigurator().Apply(configs, 50_ms);
        m_steering->GetConfigurator().Apply(configs, 50_ms);
        m_drive->GetPosition().SetUpdateFrequency(100_Hz);
        m_drive->GetVelocity().SetUpdateFrequency(100_Hz);
        m_steering->GetVelocity().SetUpdateFrequency(100_Hz);
        encoder->GetAbsolutePosition().SetUpdateFrequency(100_Hz);
    }

    void SetVelocity(complex<double> robot_velocity, double turn_rate){
        complex<double> velocity = robot_velocity + turn_vector * turn_rate;
        double wheel_speed = abs(velocity);
        angle = encoder->GetAbsolutePosition().GetValueAsDouble()*tau;
        double error = arg(velocity) - angle;
        am::wrap(error);
        if (wheel_speed < 0.008) {
            error = 0;
        }
        if (abs(error) > M_PI/2){
            error += M_PI;
            am::wrap(error);
            wheel_speed *= -1;
        }
        m_steering->Set(error/M_PI);
        // auto friction_torque =  // To account for friction, we add this to the arbitrary feed forward
        // m_steering->SetControl(torque_ctrl.WithOutput(15_A*error/M_PI*2 + friction_torque));
        auto friction_torque = 8_A/(1+exp(-wheel_speed*16))-4_A;//(wheel_speed > 0) ? 4_A : (wheel_speed < 0) ? -4_A : 0_A; // To account for friction, we add this to the arbitrary feed forward
        /* Use torque velocity */
        m_drive->SetControl(velocity_ctrl.WithVelocity(wheel_speed*motor_turns_per_m / 1_s).WithFeedForward(friction_torque));
    }

    complex<double> FindModuleVector(complex<double> robot_accel, complex<double> angular_accel) {
        return robot_accel + turn_vector * angular_accel;
    }

    complex<double> GetPositionChange() {
        angle = encoder->GetAbsolutePosition().GetValueAsDouble()*tau;
        double motor_position = m_drive->GetPosition().GetValueAsDouble();
        double motor_position_change = motor_position - motor_position_old;
        motor_position_old = motor_position;
        complex<double> position_change = polar<double>(motor_position_change / motor_turns_per_m.value(), angle);
        return position_change;
    }

    complex<double> GetVelocity() {
        angle = encoder->GetAbsolutePosition().GetValueAsDouble()*tau;
        return polar<double>(m_drive->GetVelocity().GetValueAsDouble()/motor_turns_per_m.value(), angle);
    }

    void resetEncoders() {
        motor_position_old = 0;
        m_drive->SetPosition(0_tr);
    }

private:
    hardware::TalonFX *m_drive;
    hardware::TalonFX *m_steering;
    hardware::CANcoder *encoder;
	controls::PositionTorqueCurrentFOC position_ctrl = controls::PositionTorqueCurrentFOC{0_tr}.WithSlot(1);
	controls::VelocityTorqueCurrentFOC velocity_ctrl = controls::VelocityTorqueCurrentFOC{0_tps, 0_tr_per_s_sq, 0_A, 0, false}.WithSlot(0);
	controls::TorqueCurrentFOC torque_ctrl{0_A};
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
