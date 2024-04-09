#include "TrafficLight.hpp"

void TrafficLight::bindB2Body(b2Body *body){
    b2body = body;
}

b2Body *TrafficLight::getB2Body()
{
    return b2body;
}

TrafficLightID TrafficLight::getId()
{
    return id;
}

TrafficLight::TrafficLight(const TrafficLightReport &report)
    :id(report.id), stopLine(report.stopLine){
}

void TrafficLight::report(const TrafficLightReport &report){
    allowTravel = report.allowTravel;
    timeLeft = report.timeLeft;
}

bool TrafficLight::isBindWithB2Body()
{
    return b2body != nullptr;
}
