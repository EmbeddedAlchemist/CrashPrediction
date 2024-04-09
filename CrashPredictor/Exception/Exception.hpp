#pragma once

#include <Exception/Exception.hpp>


class VehicleNotFoundException : public Exception {
public:
    inline VehicleNotFoundException(const std::string &message) :Exception(message) {};
};

class TrafficLightNotFoundException : public Exception {
public:
    inline TrafficLightNotFoundException(const std::string &message) :Exception(message) {};
};

class VehicleHasBindedWithB2BodyException :public Exception {};
class VehicleRecordNotEnoughException : public Exception {};

class VehicleNotBindWithB2BodyException : public Exception {
public:
    inline VehicleNotBindWithB2BodyException(const std::string &message) :Exception(message) {};
};