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
		/* Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
		slot0Configs.kP = 5; // An error of 1 rotation per second results in 5 amps output
		slot0Configs.kI = 0.1; // An error of 1 rotation per second increases output by 0.1 amps every second
		slot0Configs.kD = 0.001; // A change of 1000 rotation per second squared results in 1 amp output
		configs.TorqueCurrent.PeakForwardTorqueCurrent = 30;
		configs.TorqueCurrent.PeakReverseTorqueCurrent = -30;
        m_drive->GetConfigurator().Apply(configs, 50_ms);
        m_steering->GetConfigurator().Apply(configs, 50_ms);
        m_drive->GetPosition().SetUpdateFrequency(200_Hz);
        m_drive->GetVelocity().SetUpdateFrequency(200_Hz);
        m_steering->GetVelocity().SetUpdateFrequency(200_Hz);
        encoder->GetAbsolutePosition().SetUpdateFrequency(200_Hz);
    }

    void SetVelocity(complex<double> robot_velocity, double turn_rate){
        angle = encoder->GetAbsolutePosition().GetValue();
        complex<double> velocity = robot_velocity + turn_vector * turn_rate;
        double wheel_speed = abs(velocity);
        double error = arg(velocity) - angle.value();
        am::wrap(error);
        if (wheel_speed < 0.003 * constants::max_m_per_sec) {
            error = 0;
        }
        if (abs(error) > (M_PI/2)){
            error += M_PI;
            am::wrap(error);
            wheel_speed *= -1;
        }
        m_steering->Set(error/M_PI);
        auto friction_torque = (wheel_speed > 0) ? 4.5_A : -4.5_A; // To account for friction, we add this to the arbitrary feed forward
        /* Use torque velocity */
        m_drive->SetControl(velocity_ctrl.WithVelocity(wheel_speed*constants::motor_turns_per_m / 1_s).WithFeedForward(friction_torque));
    }

    void SetAcceleration(complex<double> robot_accel, double angular_accel, complex<double> robot_velocity, double angular_velocity){
        angle = encoder->GetAbsolutePosition().GetValue();
        complex<double> last_velocity = FindModuleVector(robot_velocity, angular_velocity);
        complex<double> accel = FindModuleVector(robot_accel, angular_accel);
        if (abs(last_velocity) > 0.003*constants::max_m_per_sec) {
            robot_velocity += robot_accel*constants::max_m_per_sec_per_cycle;
            angular_velocity += angular_accel*constants::max_m_per_sec_per_cycle;
            complex<double> velocity = FindModuleVector(robot_velocity, angular_velocity);
            complex<double> wheel_unit_vector = polar<double>(1, angle.value());
            double wheel_accel = accel.real()*wheel_unit_vector.real() + accel.imag()*wheel_unit_vector.imag();
            error = arg(velocity) - angle.value();
            double angle_change = arg(velocity) - arg(last_velocity);
            am::wrap(error);
            am::wrap(angle_change);
            if (abs(angle_change) > M_PI/2) {
                angle_change += M_PI;
                am::wrap(angle_change);
            }
            if (abs(error) > M_PI/2){
                error += M_PI;
                am::wrap(error);
            }
            double steering_rate = angle_change*6.4/M_PI*constants::cycle_time.value() + error*50;
            m_drive->SetControl(torque_ctrl.WithOutput(wheel_accel*constants::max_current).WithDeadband(1_A));
            auto friction_torque = (steering_rate > 0) ? 3_A : -3_A;
            m_steering->SetControl(velocity_ctrl.WithVelocity(steering_rate*1_tps).WithFeedForward(friction_torque));
        } else if (abs(accel) > 0.003) {
            complex<double> wheel_unit_vector = polar<double>(1, angle.value());
            double wheel_accel = accel.real()*wheel_unit_vector.real() + accel.imag()*wheel_unit_vector.imag();
            error = arg(accel) - angle.value();
            am::wrap(error);
            if (abs(error) > M_PI/2){
                error += M_PI;
                am::wrap(error);
            }
            double steering_rate = error*50;
            m_drive->SetControl(torque_ctrl.WithOutput(wheel_accel*constants::max_current).WithDeadband(1_A));
            auto friction_torque = (steering_rate > 0) ? 3_A : -3_A;
            m_steering->SetControl(velocity_ctrl.WithVelocity(steering_rate*1_tps).WithFeedForward(friction_torque));
        } else {
            
        }
        // frc::SmartDashboard::PutNumber("x" + to_string(modID), velocity.real());
        // frc::SmartDashboard::PutNumber("y" + to_string(modID), velocity.imag());
    }

    complex<double> FindModuleVector(complex<double> robot_accel, complex<double> angular_accel) {
        return robot_accel + turn_vector * angular_accel;
    }

    complex<double> GetVelocity(){
        angle = encoder->GetAbsolutePosition().GetValue();
        module_velocity = polar<double>(m_drive->GetVelocity().GetValueAsDouble() - m_steering->GetVelocity().GetValueAsDouble()*0.279, angle.value());
        return module_velocity;
    }

    double GetAngularVelocity() {
        return module_velocity.real()*turn_vector.real() + module_velocity.imag()*turn_vector.imag();
    }

    complex<double> GetPositionChange() {
        angle = encoder->GetAbsolutePosition().GetValue();
        double motor_position = m_drive->GetPosition().GetValueAsDouble();
        double motor_position_change = motor_position - motor_position_old;
        motor_position_old = motor_position;
        complex<double> position_change = polar<double>(motor_position_change / constants::motor_turns_per_m.value(), angle.value());
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
	controls::VelocityTorqueCurrentFOC velocity_ctrl{0_tps, 0_tr_per_s_sq, 0_A, 0, false};
	controls::TorqueCurrentFOC torque_ctrl{0_A};
    int modID;
    complex<double> turn_vector;
    complex<double> module_velocity;
    units::radian_t angle;
    double error;
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
