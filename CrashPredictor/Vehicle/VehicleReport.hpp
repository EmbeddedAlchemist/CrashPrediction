#pragma once

#include "VehicleID.hpp"
#include <ostream>
#include <ctime>


struct VehiclePosition {
    float x;
    float y;
    float angle;
};

struct VehicleVelocity {
    float linear;
    float angular;
};

struct VehicleAcceleration {
    float linear;
    float angular;
};

struct VehicleReport{
    std::time_t timestamp;
    VehicleID id;
    VehiclePosition position;
    VehicleVelocity velocity;
    VehicleAcceleration accleration;

    VehicleReport();

    friend std::ostream &operator << (std::ostream &out, const VehicleReport &report);
};

