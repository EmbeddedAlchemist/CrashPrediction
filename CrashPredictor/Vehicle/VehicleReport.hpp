#pragma once

#include "VehicleID.hpp"
#include <ostream>
#include <ctime>
#include <optional>
#include <chrono>

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
    long long timestamp; //ms
    VehicleID id;
    VehiclePosition position;
    VehicleVelocity velocity;
    std::optional<VehicleAcceleration> accleration;

    VehicleReport();

    friend std::ostream &operator << (std::ostream &out, const VehicleReport &report);
};

