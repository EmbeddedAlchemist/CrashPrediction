#pragma once

#include <cstdint>

struct TrafficLightStopLinePosition {
    float x;
    float y;
    float angle;
};

using TrafficLightStopLineWidth = float;

struct TrafficLightStopLine {
    TrafficLightStopLinePosition position;
    TrafficLightStopLineWidth width;
};

using TrafficLightID = std::uint32_t;
