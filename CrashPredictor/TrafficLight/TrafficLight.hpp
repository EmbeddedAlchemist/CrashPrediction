#pragma once

#include <cstdint>
#include <box2d/box2d.h>

#include "TrafficLightProps.hpp"
#include "TrafficLightReport.hpp"

class TrafficLight{
private:
    const int _type_mask = 2;
private:
    b2Body *b2body = nullptr;
    friend class Scene;

    void bindB2Body(b2Body *body);
    b2Body *getB2Body();

    const TrafficLightID id;
    bool allowTravel;
    float timeLeft;
    const TrafficLightStopLine stopLine;

public:

    TrafficLightID getId();

    TrafficLight(const TrafficLightReport &report);
    void report(const TrafficLightReport &report);
    bool isBindWithB2Body();

    inline bool isAllowTravel() const { return allowTravel; }
    inline float getTimeLeft() const { return timeLeft; }
    inline TrafficLightStopLine getStopLine() const { return stopLine; }

};

