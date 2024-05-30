#pragma once

#include "math.h"
#include <iostream>
#include <vector>
#include <fstream>
#include "complex.h"
#include "json.hpp"

using namespace std;

struct Sample {
    units::time::second_t timestamp;
    complex<double> position;
    double heading;
    complex<double> velocity;
    double angular_velocity;
};

class Trajectory {
    public:
        Trajectory(const std::string& filename) {
            ifstream file(filename);
            nlohmann::json jsonData;
            file >> jsonData;
            Sample sample;
            for (const auto& item : jsonData["samples"]) {
                sample.timestamp = item["timestamp"].get<double>() * 1_s;
                sample.position = complex<double>(item["x"].get<double>(), item["y"].get<double>());
                sample.heading = item["heading"].get<double>();
                sample.velocity = complex<double>(item["velocityX"].get<double>(), item["velocityY"].get<double>());
                sample.angular_velocity = item["angularVelocity"].get<double>();
                samples.push_back(sample);
            }
            end_time = sample.timestamp;
        }

        Sample GetSample(int index) {
            return samples[index];
        }

        int GetSampleCount() {
            return samples.size();
        }

        units::time::second_t GetEndTime() {
            return end_time;
        }
    private:
        vector<Sample> samples;         // sample list
        units::time::second_t end_time; // time it takes to complete the trajectory
};