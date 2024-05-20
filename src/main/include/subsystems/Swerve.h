#pragma once

#include <AHRS.h>
#include "subsystems/SwerveModule.h"
#include "trajectoryMaker.h"

using namespace std;

class Swerve{
public:
    
    // drives robot at given speed during teleop
    void set(complex<double> velocity, double turn_rate){
        heading = -gyro.GetYaw()*(M_PI/180);
        target_velocity = velocity;
        // robot orient the velocity
        velocity *= polar<double>(1, -heading);
        // find fastest module speed
        fastest = constants::max_speed_meters_per_second;
        for (Module& module : modules){
            module_speed = abs(module.getVelocity(velocity, turn_rate));
            if (module_speed > fastest)
                fastest = module_speed;
        }
        // move current velocity toward target
        target_velocity *= constants::max_speed_meters_per_second / fastest;
        turn_rate /= fastest;
        velocity_error = target_velocity-current_velocity;
        turn_rate_error = turn_rate - current_turn_rate;
        if (abs(velocity_error) > constants::slew_rate) {
            velocity_error *= constants::slew_rate/abs(velocity_error);
        }
        if (abs(turn_rate_error) > constants::slew_rate) {
            turn_rate_error *= constants::slew_rate/abs(turn_rate_error);
        }
        current_velocity += velocity_error;
        current_turn_rate += turn_rate_error;
        // robot orient velocity
        target_velocity = current_velocity * polar<double>(1, -heading);
        // calculate odometry and drive the modules
        position_change = complex<double>(0,0);
        for (Module& module : modules) {
            module.set(target_velocity, current_turn_rate);
            position_change += module.getPositionChange();
        }
        position += position_change * polar<double>(0.25, heading);
    }

    // drive toward the position setpoint with feedforward
    void SetPose(trajectoryMaker::Sample sample) {
        // calculate proporional response
        heading = -gyro.GetYaw()*(M_PI/180);
        position_error = sample.position - position;
        heading_error = sample.heading - heading;
        am::wrap(heading_error);
        sample.velocity += position_P * position_error;
        sample.angular_velocity += heading_P * heading_error;
        //robot orient the output
        sample.velocity *= polar<double>(1, -heading);
        // calculate odometry and drive modules
        position_change = complex<double>(0,0);
        for (Module& module : modules){
            module.set(sample.velocity, sample.angular_velocity);
            position_change += module.getPositionChange();
        }
        position += position_change * polar<double>(0.25, heading);
    }

    void init(){
        for (Module& module : modules){
            module.init();
        }
    }

    void resetPos(complex<double> new_position = complex<double>(0,0)) {
        for (Module& module : modules) {
            module.resetEncoders();
        }
        position = new_position;
    }

    complex<double> GetPosition() {
        return position;
    }

private:
    AHRS gyro{frc::SPI::Port::kMXP};
    Module modules[4] = {
        Module{1, complex<double>(1, 1)},
        Module{2, complex<double>(-1, 1)},
        Module{3, complex<double>(1, -1)},
        Module{4, complex<double>(-1, -1)}
    };

      // heading proportional response rate
    complex<double> current_velocity; // current velocity the swerve is set to in teleop
    double current_turn_rate = 0;     // current turn rate of the swerve in teleop
    double heading;
    complex<double> target_velocity;
    complex<double> velocity_error;
    double turn_rate_error;
    double module_speed;
    double fastest;


    complex<double> position = complex<double>(0, 0); // current position of the robot
    complex<double> position_change;
    double position_P = 0.04;         // position proportional response rate
    double heading_P = 2.5;

    complex<double> position_error;
    double heading_error;
} swerve;
