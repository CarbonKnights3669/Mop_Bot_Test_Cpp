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
    const units::meters_per_second_squared_t max_acceleration = 4_mps_sq;
    const units::ampere_t max_current = 25_A;
    const units::second_t cycle_time = 5_ms;
    const double max_m_per_sec_per_cycle = max_acceleration.value() * cycle_time.value();    // how quickly to change the swerve velocity
    const units::turn_t motor_turns_per_wheel_turn = 6.12_tr;
    const units::meter_t wheel_diameter = 3.9_in;
    const units::turn_t motor_turns_per_m = 1_m / (wheel_diameter * M_PI) * motor_turns_per_wheel_turn;
    const double max_m_per_sec = 5;
}