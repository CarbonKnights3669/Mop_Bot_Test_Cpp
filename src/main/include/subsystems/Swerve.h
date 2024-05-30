#pragma once

#include <AHRS.h>
#include "subsystems/SwerveModule.h"
#include "utils/Trajectory.h"

using namespace std;

class Swerve{
public:
    // drives robot at given speed during teleop
    void SetAcceleration(double x_accel, double y_accel, double angular_accel, double rate_modifier = 1){
        complex<double> accel = complex<double>(x_accel, y_accel);
        // apply smooth deadband
        double dB = 0.03;
        accel = (abs(accel)>dB) ? accel*(1 - dB/abs(accel))/(1-dB) : 0;
        angular_accel = (abs(angular_accel)>dB) ? angular_accel*(1 - dB/abs(angular_accel))/(1-dB) : 0;
        // robot orient the acceleration
        heading = -gyro.GetYaw()*(M_PI/180);
        accel *= polar<double>(1, -heading);
        // find fastest module speed
        double greatest = 1;
        for (auto& module : modules){
            double module_accel = abs(module.FindModuleVector(accel, angular_accel));
            if (module_accel > greatest)
                greatest = module_accel;
        }
        // limit output so no module goes above 1
        accel /= greatest;
        angular_accel /= greatest;
        frc::SmartDashboard::PutNumber("x accel", accel.real());
        frc::SmartDashboard::PutNumber("y accel", accel.imag());
        frc::SmartDashboard::PutNumber("angular accel", angular_accel);
        // calculate current velocity
        complex<double> velocity;
        double angular_velocity = 0;
        for (auto& module : modules) {
            velocity += module.GetVelocity();
            angular_velocity += module.GetAngularVelocity();
        }
        velocity *= 0.25;
        angular_velocity *= 0.25;
        // frc::SmartDashboard::PutNumber("x vel", velocity.real());
        // frc::SmartDashboard::PutNumber("y vel", velocity.imag());
        // frc::SmartDashboard::PutNumber("angular vel", angular_velocity);
        for (auto& module : modules) {
            // module.SetVelocity(accel, angular_accel);
            module.SetAcceleration(accel, angular_accel, velocity, angular_velocity);
        }
    }

    // sets the trajectory to follow
    void SetTrajectory(Trajectory &trajectory) {
        this->trajectory = &trajectory;
        sample_index = 0;
    }

    // todo: add acceleration feedforward

    // drive toward the position setpoint with feedforward and return true when done
    void FollowTrajectory() {
        heading = -gyro.GetYaw()*(M_PI/180);
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
            am::wrap(heading_error);
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

    void SetModuleVelocities(complex<double> acceleration = complex<double>(0,0), double angular_velocity = 0) {
        //robot orient the acceleration
        acceleration *= polar<double>(1, -heading);
        for (auto& module : modules){
            module.SetVelocity(acceleration, angular_velocity);
        }
    }

    AHRS gyro{frc::SPI::Port::kMXP};
    frc::Timer auto_timer;
    vector<SwerveModule> modules;

    Trajectory *trajectory;    // trajectory currently being followed in autonomous
    double current_turn_rate = 0;     // current turn rate of the swerve in teleop
    double heading;
    int sample_index = 0;

    complex<double> position = complex<double>(0, 0); // current position of the robot
    double position_P = 0.04;         // position proportional response rate
    double heading_P = 2.5;           // heading proportional response rate
};
