#include "HTTPInterface.hpp"
#include "ExLog/ExLog.hpp"

using namespace ArduinoJson;

void HTTPInterface::registerInterface() {
    server.Options(R"(/.*)", [](const httplib::Request &req, httplib::Response &res) {
        setDefaultResponseHeader(res);
        LOG_V << "aca";
        res.status = 200;
        });

    server.Get("/get_info", [&](const httplib::Request &req, httplib::Response &res) {
        setDefaultResponseHeader(res);
        JsonDocument response;
        response["error_status"] = "OK";
        //LOG_I << "get_info request from " << req.remote_addr << ':' << req.remote_port;


        try {
            //praseGetInfoReq(request, response);
            std::string type = getRequestParam(req, "type");
            if (type == "vehicle_list") getVehicleList(response);
            else if (type == "vehicle_info") getVehicleInfo(response, getRequestParam(req, "id"));
            else if (type == "all_vehicle_info") throw Exception("deprecated"); //getAllVehicleInfo(response);
            else if (type == "all_traffic_light_info") throw Exception("deprecated"); //getAllTrafficLightInfo(response);
            else if (type == "all_info") getAllInfo(response);
            else throw Exception("bad_type");
        }
        catch (Exception err) {
            LOG_W << err.message;
            response["error_status"] = err.message;
            res.status = 400;
        }

        std::string resStr;
        serializeJson(response, resStr);
        res.set_content(resStr, "application/json");

        });

    server.Post("/post_report", [&](const httplib::Request &req, httplib::Response &res) {
        setDefaultResponseHeader(res);
        JsonDocument request;
        JsonDocument response;
        response["error_status"] = "OK";
        LOG_I << "post-report request from " << req.remote_addr << ':' << req.remote_port;
        try {
            auto desErr = deserializeJson(request, req.body);
            if (desErr != DeserializationError::Ok)
                throw Exception(std::string("Format Error: ") + desErr.c_str());
            std::string type = getRequestParam(req, "type");
            if (type == "vehicle_register")
                vehicleRegister(request, response);
            else if (type == "vehicle_report")
                vehicleReport(request, response);
            else if (type == "vehicle_unregister")
                vehicleUnregister(request, response);
            else if (type == "traffic_light_report")
                trafficLightReport(request, response);
            else
                throw Exception("bad_type");
        }
        catch (Exception err) {
            LOG_W << err.message;
            response["error_status"] = err.message;
            res.status = 400;
        }

        std::string resStr;
        serializeJson(response, resStr);
        res.set_content(resStr, "application/json");
        });


    server.Post("/debug", [](const httplib::Request &req, httplib::Response &res) {
        setDefaultResponseHeader(res);

        });

    server.set_error_handler([](const httplib::Request &req, httplib::Response &res) {
        if (!res.body.empty())
            return;
        setDefaultResponseHeader(res);
        res.set_content("{\"err_status\":\"bad_entry\"}", "application/json");
        
        res.status = 400;
        });

    LOG_I << "HTTP Interface has registered";


}

std::string HTTPInterface::getRequestParam(const httplib::Request &req, const std::string name) {
    auto iter = req.params.find(name);
    if (iter == req.params.end())
        throw Exception((std::ostringstream() << "Param '" << name << "' not found").str());
    return iter->second;
}

void HTTPInterface::setDefaultResponseHeader(httplib::Response &res){
    res.set_header("Access-Control-Allow-Origin", "*");
    res.set_header("Access-Control-Allow-Methods", "GET,POST,PUT,DELETE,PATCH,OPTIONS");
    res.set_header("Access-Control-Allow-Headers", "Content-Type,Accept,Authorization");
    res.set_header("Access-Control-Max-Age", "1728000");
}



void HTTPInterface::getVehicleList(ArduinoJson::JsonDocument &response) {
    auto res = response["vehicle_list"].to<JsonArray>();
    auto list = predictor.getVehicleList();
    for (auto &veh : list) {
        JsonObject jveh = res.add<JsonObject>();
        jveh["id"] = veh->getId();
    }
}

void HTTPInterface::getVehicleInfo(ArduinoJson::JsonDocument &response, const std::string &s_id){
    VehicleID id = std::stoul(s_id);
    std::shared_ptr<Vehicle> vehicle;
    vehicle = predictor.findVehicleById(id);
    auto jInfo = response["info"].to<JsonObject>();
    //auto jInfo = res["vehicle_info"].to<JsonObject>();
    jInfo["error_status"] = "OK";
    jInfo["id"] = id;
    try {
        auto position = vehicle->getPosition();
        auto jPosition = jInfo["position"].to<JsonObject>();
        jPosition["x"] = position.x;
        jPosition["y"] = position.y;
        jPosition["angle"] = position.angle;
        auto velocity = vehicle->getVelocity();
        auto jVelocity = jInfo["velocity"].to<JsonObject>();
        jVelocity["linear"] = velocity.linear;
        jVelocity["angular"] = velocity.angular;
        auto jAccleration = jInfo["accletion"].to<JsonObject>();
        auto accleration = vehicle->getAccleration();
        jAccleration["linear"] = accleration.linear;
        jAccleration["angular"] = accleration.angular;

        auto predictStatus = vehicle -> getPredictStatus();
        auto jPredictStatus = jInfo["predict_status"].to<JsonObject>();
        if (predictStatus.contact.has_value()) {
            auto jContact = jPredictStatus["contact"].to<JsonObject>();
            jContact["id"] = predictStatus.contact->id;
            jContact["reason"] = predictStatus.contact->reason;
        }
        else
            jPredictStatus["contact"] = nullptr;
        if (predictStatus.recommendSpeed.has_value())
            jPredictStatus["recommend_speed"] = predictStatus.recommendSpeed.value();
        else
            jPredictStatus["recommend_speed"] = nullptr;
    }
    catch (VehicleNotBindWithB2BodyException e) {
        jInfo["error_status"] = e.message;
    }
}

//void HTTPInterface::getAllVehicleInfo(ArduinoJson::JsonDocument &res){
//    auto vehicleList = predictor.getVehicleList();
//    auto jList = res["infos"].to<JsonArray>();
//    for (auto &veh : vehicleList) {
//        auto jInfo = jList.add<JsonObject>();
//        jInfo["error_status"] = "OK";
//        jInfo["id"] = veh->getId();
//        try {
//            auto size = veh->getSize();
//            auto jSize = jInfo["size"].to<JsonObject>();
//            jSize["width"] = size.width;
//            jSize["length"] = size.length;
//            jInfo["weight"] = veh->getWeight();
//            auto position = veh->getPosition();
//            auto jPosition = jInfo["position"].to<JsonObject>();
//            jPosition["x"] = position.x;
//            jPosition["y"] = position.y;
//            jPosition["angle"] = position.angle;
//            auto velocity = veh->getVelocity();
//            auto jVelocity = jInfo["velocity"].to<JsonObject>();
//            jVelocity["linear"] = velocity.linear;
//            jVelocity["angular"] = velocity.angular;
//            auto jAccleration = jInfo["accletion"].to<JsonObject>();
//            auto accleration = veh->getAccleration();
//            jAccleration["linear"] = accleration.linear;
//            jAccleration["angular"] = accleration.angular;
//        }
//        catch (VehicleNotBindWithB2BodyException e) {
//            jInfo["error_status"] = e.message;
//        }
//    }
//    
//}

//void HTTPInterface::getAllTrafficLightInfo(ArduinoJson::JsonDocument &res){
//    auto trafficLightList = predictor.getTrafficLightList();
//    auto jList = res["infos"].to<JsonArray>();
//    for (auto &tl : trafficLightList) {
//        auto jInfo = jList.add<JsonObject>();
//        jInfo["id"] = tl->getId();
//        jInfo["allow_travel"] = tl->isAllowTravel();
//        jInfo["time_left"] = tl->getTimeLeft();
//        auto stopLine = tl->getStopLine();
//        auto jStopLine = jInfo["stop_line"].to<JsonObject>();
//        auto jPosition = jStopLine["position"].to<JsonObject>();
//        jPosition["x"] = stopLine.position.x;
//        jPosition["y"] = stopLine.position.y;
//        jPosition["angle"] = stopLine.position.angle;
//        jStopLine["width"] = stopLine.width;
//
//    }
//}

void HTTPInterface::getAllInfo(ArduinoJson::JsonDocument &res){
    {
        auto vehicleList = predictor.getVehicleList();
        auto jList = res["vehicles_infos"].to<JsonArray>();
        for (auto &veh : vehicleList) {
            auto jInfo = jList.add<JsonObject>();
            jInfo["error_status"] = "OK";
            jInfo["id"] = veh->getId();
            try {
                auto size = veh->getSize();
                auto jSize = jInfo["size"].to<JsonObject>();
                jSize["width"] = size.width;
                jSize["length"] = size.length;
                jInfo["weight"] = veh->getWeight();
                auto position = veh->getPosition();
                auto jPosition = jInfo["position"].to<JsonObject>();
                jPosition["x"] = position.x;
                jPosition["y"] = position.y;
                jPosition["angle"] = position.angle;
                auto velocity = veh->getVelocity();
                auto jVelocity = jInfo["velocity"].to<JsonObject>();
                jVelocity["linear"] = velocity.linear;
                jVelocity["angular"] = velocity.angular;
                auto jAccleration = jInfo["accletion"].to<JsonObject>();
                auto accleration = veh->getAccleration();
                jAccleration["linear"] = accleration.linear;
                jAccleration["angular"] = accleration.angular;

                auto predictStatus = veh->getPredictStatus();
                auto jPredictStatus = jInfo["predict_status"].to<JsonObject>();
                if (predictStatus.contact.has_value()) {
                    auto jContact = jPredictStatus["contact"].to<JsonObject>();
                    jContact["id"] = predictStatus.contact->id;
                    jContact["reason"] = predictStatus.contact->reason;
                }
                else
                    jPredictStatus["contact"] = nullptr;
                if (predictStatus.recommendSpeed.has_value())
                    jPredictStatus["recommend_speed"] = predictStatus.recommendSpeed.value();
                else
                    jPredictStatus["recommend_speed"] = nullptr;
            }
            catch (VehicleNotBindWithB2BodyException e) {
                jInfo["error_status"] = e.message;
            }
        }
    }
    {
        auto trafficLightList = predictor.getTrafficLightList();
        auto jList = res["traffic_lights_infos"].to<JsonArray>();
        for (auto &tl : trafficLightList) {
            auto jInfo = jList.add<JsonObject>();
            jInfo["id"] = tl->getId();
            jInfo["allow_travel"] = tl->isAllowTravel();
            jInfo["time_left"] = tl->getTimeLeft();
            auto stopLine = tl->getStopLine();
            auto jStopLine = jInfo["stop_line"].to<JsonObject>();
            auto jPosition = jStopLine["position"].to<JsonObject>();
            jPosition["x"] = stopLine.position.x;
            jPosition["y"] = stopLine.position.y;
            jPosition["angle"] = stopLine.position.angle;
            jStopLine["width"] = stopLine.width;

        }
    }
}

void HTTPInterface::vehicleRegister(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res){
    auto jInfo = getJsonProp<JsonObjectConst>(req, "vehicle_info");
    VehicleInfo info;
    info.id = getJsonProp<VehicleID>(jInfo, "id");
    auto jSize = getJsonProp<JsonObjectConst>(jInfo, "size");
    info.size.width = getJsonProp<float>(jSize, "width");
    info.size.length = getJsonProp<float>(jSize, "length");
    info.weight = getJsonProp<float>(jInfo, "weight");
    info.maxBrakeForce = getJsonProp<float>(jInfo, "max_brake_force");
    predictor.registerVehicle(info);
}

void HTTPInterface::vehicleReport(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res){
    auto jReport = getJsonProp<JsonObjectConst>(req, "report");
    VehicleReport report;
    report.id = getJsonProp<VehicleID>(jReport, "id");
    auto jPosition = getJsonProp<JsonObjectConst>(jReport, "position");
    report.position.x = getJsonProp<float>(jPosition, "x");
    report.position.y = getJsonProp<float>(jPosition, "y");
    report.position.angle = getJsonProp<float>(jPosition, "angle");
    auto jVelocity = getJsonProp<JsonObjectConst>(jReport, "velocity");
    report.velocity.linear = getJsonProp<float>(jVelocity, "linear");
    report.velocity.angular = getJsonProp<float>(jVelocity, "angular");
    auto jAccleration = getJsonProp<JsonObjectConst>(jReport, "accleration");
    report.accleration.linear = getJsonProp<float>(jAccleration, "linear");
    report.accleration.angular = getJsonProp<float>(jAccleration, "angular");
    predictor.vehicleReport(report);
}

void HTTPInterface::vehicleUnregister(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res){
    VehicleID id = getJsonProp<VehicleID>(req, "id");
    predictor.unregisterVehicle(id);
}

void HTTPInterface::trafficLightReport(const ArduinoJson::JsonDocument &req, ArduinoJson::JsonDocument &res){
    TrafficLightReport report;
    auto jReport = getJsonProp<JsonObjectConst>(req, "report");
    report.id = getJsonProp<TrafficLightID>(jReport, "id");
    report.allowTravel = getJsonProp<bool>(jReport, "allow_travel");
    report.timeLeft = getJsonProp<float>(jReport, "time_left");
    auto jStopLine = getJsonProp<JsonObjectConst>(jReport, "stop_line");
    auto jPosition = getJsonProp<JsonObjectConst>(jStopLine, "position");
    report.stopLine.position.x = getJsonProp<float>(jPosition, "x");
    report.stopLine.position.y = getJsonProp<float>(jPosition, "y");
    report.stopLine.position.angle = getJsonProp<float>(jPosition, "angle");
    report.stopLine.width = getJsonProp<float>(jStopLine, "width");
    predictor.trafficLightReport(report);
}


HTTPInterface::HTTPInterface(CrashPredictor &predictor)
    :predictor(predictor) {
}

void HTTPInterface::begin(const std::string &ip, std::uint16_t port) {
    server.set_logger([](const httplib::Request &req, const httplib::Response &res) {
        //LOG_V << req.method << ' ' << req.path << " from " << req.remote_addr << ':' << req.remote_port;
        //LOG_V << "Response:" << res.body;
        });

    registerInterface();
    if (not server.listen(ip, port)) {
        LOG_E << "Fail to listen " << ip << ':' << port;
        throw Exception("fail to bind " + port);
    }
}
