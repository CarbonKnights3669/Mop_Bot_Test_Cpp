#pragma once

#include <math.h>
#include <units/math.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>

namespace constants {
    // how often to run the robot program
    const units::second_t cycle_time = 10_ms;
    // maximum module current
    const double max_current = 25;
    // current required to account for friction
    const double feedforward_current = 4;
    // current headroom
    const double current_headroom = 4;
    // maximum accelerations in meters per second squared
    const double max_accel = 9;
    // number of amps to achieve 1mps^2 acceleration
    const double current_to_accel_ratio = 10;
    // how quickly to change the swerve velocity
    const double max_m_per_sec_per_cycle = max_accel * cycle_time.value();
    const double motor_turns_per_wheel_turn = 6.12;
    const units::meter_t wheel_diameter = 3.9_in;
    const double motor_turns_per_m = motor_turns_per_wheel_turn / (wheel_diameter.value() * M_PI);
    const units::meter_t furthest_module_center_dist = 0.305_m * sqrt(2);
    const double max_m_per_sec = 5;


    const double tau = 2*M_PI;
}