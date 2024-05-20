#pragma once
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <string>
#include <complex>

using namespace std;

class Limelight {
public:
    double getSpeakerYaw() {
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        return table->GetNumber("tx", 0.0);
    }

    double getSpeakerPitch() {
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        return table->GetNumber("ty", 0.0);
    }

    bool getTargetValid() {
        std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        return table->GetNumber("tx", 0.0) != 0;
    }
} ll;