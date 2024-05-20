#pragma once

#include "math.h"
#include <iostream>
#include <vector>
#include <fstream>
#include "complex.h"
#include "json.hpp"

using namespace std;
namespace trajectoryMaker
{
    struct Sample {
        units::time::second_t timestamp;
        complex<double> position;
        double heading;
        complex<double> velocity;
        double angular_velocity;
    };

    // Function to load JSON data into a vector of Sample structures
    vector<Sample> MakeTrajectory(const std::string& filename) {
        ifstream file(filename);
        nlohmann::json jsonData;
        file >> jsonData;

        vector<Sample> trajectory;

        for (const auto& item : jsonData["samples"]) {
            Sample sample;
            sample.timestamp = item["timestamp"].get<double>() * 1_s;
            sample.position = complex<double>(item["x"].get<double>(), item["y"].get<double>());
            sample.heading = item["heading"].get<double>();
            sample.velocity = complex<double>(item["velocityX"].get<double>(), item["velocityY"].get<double>());
            sample.angular_velocity = item["angularVelocity"].get<double>();
            trajectory.push_back(sample);
        }

        return trajectory;
    }
}