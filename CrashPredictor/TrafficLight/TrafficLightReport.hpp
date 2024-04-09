#pragma once

#include "TrafficLightProps.hpp"


struct TrafficLightReport{
    TrafficLightID id;
    bool allowTravel;
    float timeLeft;
    TrafficLightStopLine stopLine;
};

