#pragma once

#include <string>
#include <cstdint>
#include <httplib/httplib.h>
#include <CrashPredictor/CrashPredictor.hpp>
#include <ArduinoJson/ArduinoJson.hpp>
#include <Exception/Exception.hpp>


class HTTPInterface{


private:
    httplib::Server  server;
    CrashPredictor &predictor;

private:
    void registerInterface();
    static std::string getRequestParam(const httplib::Request &req, const std::string name);
    template <typename Type ,typename _JsonType>
    static Type getJsonProp(_JsonType &obj, const std::string &name);
    static void setDefaultResponseHeader(httplib::Response &res);

private:
    void getVehicleList(ArduinoJson::JsonDocument &res);
    void getVehicleInfo(ArduinoJson::JsonDocument &res, const std::string &id);
    //void getAllVehicleInfo(ArduinoJson::JsonDocument &res);
    //void getAllTrafficLightInfo(ArduinoJson::JsonDocument &res);
    void getAllInfo(ArduinoJson::JsonDocument &res);

    void vehicleRegister(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res);
    void vehicleReport(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res);
    void vehicleUnregister(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res);

    void trafficLightReport(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res);

public:
    HTTPInterface(CrashPredictor &predictor);
    void begin(const std::string &ip, std::uint16_t port);
};

template<typename Type, typename _JsonType>
inline Type HTTPInterface::getJsonProp(_JsonType &obj, const std::string &name){
    if (not obj.containsKey(name))
        throw Exception((std::ostringstream() << "Properity '" << name <<"' not found.").str());
    if (not obj[name].is<Type>())
        throw Exception((std::ostringstream() << "Properity '" << name << "' type error, expect " << typeid(Type).name() << '.').str());
    return obj[name].as<Type>();
}
