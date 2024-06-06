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
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 30;
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -30;
        m_drive->GetConfigurator().Apply(configs, 50_ms);
        m_steering->GetConfigurator().Apply(configs, 50_ms);
        m_drive->GetPosition().SetUpdateFrequency(100_Hz);
        m_drive->GetVelocity().SetUpdateFrequency(100_Hz);
        m_steering->GetVelocity().SetUpdateFrequency(100_Hz);
        encoder->GetAbsolutePosition().SetUpdateFrequency(100_Hz);
    }

    void SetVelocity(complex<double> robot_velocity, double turn_rate){
        angle = encoder->GetAbsolutePosition().GetValueAsDouble();
        complex<double> velocity = robot_velocity + turn_vector * turn_rate;
        double wheel_speed = abs(velocity);
        double error = arg(velocity)/tau - angle;
        am::wrapTurns(error);
        if (wheel_speed < 0.003 * max_m_per_sec) {
            error = 0;
        }
        if (abs(error) > 0.25){
            error += 0.5;
            am::wrapTurns(error);
            wheel_speed *= -1;
        }
        m_steering->Set(error*2);
        auto friction_torque = (wheel_speed > 0) ? 4.5_A : -4.5_A; // To account for friction, we add this to the arbitrary feed forward
        /* Use torque velocity */
        m_drive->SetControl(velocity_ctrl.WithVelocity(wheel_speed*motor_turns_per_m / 1_s).WithFeedForward(friction_torque));
    }

    void SetAcceleration(complex<double> robot_accel, double angular_accel, complex<double> robot_velocity, double angular_velocity) {
        angle = encoder->GetAbsolutePosition().GetValueAsDouble()*tau;
        complex<double> velocity = FindModuleVector(robot_velocity, angular_velocity);
        complex<double> accel = FindModuleVector(robot_accel, angular_accel);
        double wheel_accel = accel.real()*cos(angle) + accel.imag()*sin(angle);
        complex<double> next_velocity = velocity + accel*max_m_per_sec_per_cycle;
        // if (abs(velocity) > 0.02) {
        //     double error = arg(next_velocity) - angle;
        //     am::wrap(error);
        //     if (abs(error) > tau/4){
        //         error += tau/2;
        //         am::wrap(error);
        //     }
        //     m_steering->SetControl(torque_ctrl.WithOutput(error*17_A));
        // }
        if (abs(accel) > 0.0003) {
            double error = arg(accel) - angle;
            am::wrap(error);
            if (abs(error) > tau/4){
                error += tau/2;
                am::wrap(error);
            }
            m_steering->SetControl(torque_ctrl.WithOutput(error*10_A));
        }
        else {
            m_steering->SetControl(velocity_ctrl.WithVelocity(0_tps));
        }
        m_drive->SetControl(torque_ctrl.WithOutput(wheel_accel*constants::max_current));
    }

    complex<double> FindModuleVector(complex<double> robot_accel, complex<double> angular_accel) {
        return robot_accel + turn_vector * angular_accel;
    }

    complex<double> GetPositionChange() {
        angle = encoder->GetAbsolutePosition().GetValueAsDouble()*tau;
        double motor_position = m_drive->GetPosition().GetValueAsDouble();
        double motor_position_change = motor_position - motor_position_old;
        motor_position_old = motor_position;
        complex<double> position_change = polar<double>(motor_position_change / constants::motor_turns_per_m.value(), angle);
        return position_change;
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
