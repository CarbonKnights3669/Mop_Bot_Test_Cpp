#pragma once

#include <ctre/phoenix6/Pigeon2.hpp>
#include "subsystems/SwerveModule.h"
#include "utils/Trajectory.h"

using namespace std;
using namespace ctre::phoenix6;
using namespace constants;

class Swerve{
public:
    void SetVelocity(double x_velocity, double y_velocity, double angular_velocity) {
        complex<double> velocity = complex<double>(x_velocity, y_velocity);
        // apply smooth deadband
        double dB = 0.03;
        velocity = (abs(velocity)>dB) ? velocity*(1 - dB/abs(velocity))/(1-dB) : 0;
        angular_velocity = (abs(angular_velocity)>dB) ? angular_velocity*(1 - dB/abs(angular_velocity))/(1-dB) : 0;
        // scale the velocities to meters per second
        velocity *= max_m_per_sec;
        angular_velocity *= max_m_per_sec;
        // find the robot oriented velocity
        heading = gyro.GetYaw().GetValueAsDouble()*tau/360;
        complex<double> robot_velocity = velocity * polar<double>(1, -heading);
        // find fastest module speed
        double highest = max_m_per_sec;
        for (auto& module : modules){
            double module_speed = abs(module.FindModuleVector(robot_velocity, angular_velocity));
            if (module_speed > highest)
                highest = module_speed;
        }
        // scale velocities
        velocity *= max_m_per_sec/highest;
        angular_velocity *= max_m_per_sec/highest;
        robot_velocity *= max_m_per_sec/highest;
        // find error between command and current velocities
        complex<double> vel_error = velocity - slew_vel;
        double angular_vel_error = angular_velocity - slew_angular_vel;
        // find robot oriented velocity error
        complex<double> robot_vel_error = vel_error*polar<double>(1, -heading);
        // find robot oriented slew velocity
        complex<double> robot_slew_vel = slew_vel*polar<double>(1, -heading);
        // find max acceleration overshoot
        highest = 1;
        for (auto& module : modules){
            double module_overshoot = module.GetAccelOvershoot(robot_slew_vel, slew_angular_vel, robot_vel_error, angular_vel_error);
            if (module_overshoot > highest)
                highest = module_overshoot;
        }
        // find velocity increments
        complex<double> vel_increment = vel_error/highest;
        complex<double> robot_vel_increment = robot_vel_error/highest;
        double angular_vel_increment = angular_vel_error/highest;
        // increment velocity
        if (abs(vel_error) > max_m_per_sec_per_cycle) {
            slew_vel += vel_increment;
        } else slew_vel = velocity;
        if (abs(angular_vel_error) > max_m_per_sec_per_cycle) {
            slew_angular_vel += angular_vel_increment;
        } else slew_angular_vel = angular_velocity;
        // update the robot oriented slew velocity
        robot_slew_vel = slew_vel*polar<double>(1, -heading);
        // find acceleration feedforeward
        complex<double> robot_accel = robot_vel_error*2.0;
        double angular_accel = angular_vel_error*2;
        // drive the modules
        for (auto& module : modules) {
            module.SetVelocity(robot_slew_vel, slew_angular_vel, robot_accel, angular_accel);
        }
    }

    void SetAcceleration(double x_accel, double y_accel, double angular_accel) {
        
    }

    // sets the trajectory to follow
    void SetTrajectory(Trajectory &trajectory) {
        this->trajectory = &trajectory;
        sample_index = 0;
    }

    // todo: add acceleration feedforward

    // drive toward the position setpoint with feedforward and return true when done
    void FollowTrajectory() {
        heading = gyro.GetYaw().GetValueAsDouble()*M_PI/180;
        CalculateOdometry();
        // find the latest sample index
        while (auto_timer.HasElapsed(trajectory->GetSample(sample_index).timestamp) && sample_index < trajectory->GetSampleCount()) {
            sample_index++;
        }
        if (sample_index < trajectory->GetSampleCount()) {
            Sample current_sample = trajectory->GetSample(sample_index);
            // calculate proporional response
            complex<double> position_error = current_sample.position - position;
            double heading_error = current_sample.heading - heading;
            mf::wrap(heading_error);
            current_sample.velocity += position_P * position_error;
            current_sample.angular_velocity += heading_P * heading_error;
            // drive modules
            SetModuleVelocities(current_sample.velocity, current_sample.angular_velocity);
        } else {
            SetModuleVelocities();
        }
    }

    

    void AutonomousInit() {
        for (auto& module : modules) {
            module.resetEncoders();
        }
        position = complex<double>(0,0);
        auto_timer.Restart();
    }

    units::time::second_t GetTrajectoryRemainingTime() {
        return trajectory->GetEndTime() - auto_timer.Get();
    }

    void Init(){
        for (auto& module : modules){
            module.init();
        }
        gyro.GetYaw().SetUpdateFrequency(100_Hz);
    }

    void AddModules(vector<SwerveModule> modules) {
        this->modules = modules;
    }

    void ResetPosition(complex<double> new_position = complex<double>(0,0)) {
        for (auto& module : modules) {
            module.resetEncoders();
        }
        position = new_position;
    }

    void ResetAngle() {
        gyro.SetYaw(0_deg);
    }

    complex<double> GetPosition() {
        return position;
    }

private:
    void CalculateOdometry() {
        // calculate odometry and drive modules
        complex<double> position_change;
        for (auto& module : modules){
            position_change += module.GetPositionChange();
        }
        position += position_change * polar<double>(0.25, heading);
    }

    void SetModuleVelocities(complex<double> velocity = complex<double>(0,0), double angular_velocity = 0) {
        //robot orient the acceleration
        velocity *= polar<double>(1, -heading);
        for (auto& module : modules){
            module.SetVelocity(velocity, angular_velocity);
        }
    }

    hardware::Pigeon2 gyro{1, "CTREdevices"};
    frc::Timer auto_timer;
    vector<SwerveModule> modules;

    Trajectory *trajectory;    // trajectory currently being followed in autonomous
    complex<double> slew_vel;        // current velocity of the swerve in teleop
    double slew_angular_vel = 0;     // current turn rate of the swerve in teleop
    double heading;
    int sample_index = 0;

    complex<double> position = complex<double>(0, 0); // current position of the robot
    double position_P = 0.04;         // position proportional response rate
    double heading_P = 2.5;           // heading proportional response rate
};
