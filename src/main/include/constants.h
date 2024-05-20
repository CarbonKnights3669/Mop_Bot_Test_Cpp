#pragma once

#include <math.h>
namespace constants{
    const double slew_rate = 0.03;    // how quickly to change the swerve velocity
    const double module_gear_ratio = 6.12;
    const double rotations_per_meter = module_gear_ratio / (0.09906 * M_PI);
    const double robot_loop_speed = 0.001;
    const double max_speed_meters_per_second = 5;
}