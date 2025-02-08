
#include "CrashPredictor.hpp"
#include "Exception/Exception.hpp"
#include "Scene/CrashInfo/CrashInfo.hpp"
#include "Scene/PredictOption/PredictOption.hpp"
#include "Scene/Scene.hpp"
#include "Vehicle/Vehicle.hpp"
#include "Vehicle/VehicleID.hpp"
#include "Vehicle/VehicleInfo.hpp"
#include <ExLog/ExLog.hpp>
#include <memory>
#include <vector>





std::vector<std::shared_ptr<Vehicle>>::iterator CrashPredictor::findVehicleIterByID(VehicleID id)
{
    std::lock_guard<std::mutex> lockgrand(vehicleListMutex);
    std::vector<std::shared_ptr<Vehicle>>::iterator result = std::find_if(vehicleList.begin(), vehicleList.end(), [&](std::shared_ptr<Vehicle> &item) { return item->getId() == id; });
    if (result == vehicleList.end()) throw VehicleNotFoundException((std::ostringstream() << "Vehicle with id " << id << " not found").str().c_str());
    return result;
}

std::shared_ptr<Vehicle> CrashPredictor::findVehicleById(VehicleID id)
{
    return *findVehicleIterByID(id);
}

std::shared_ptr<TrafficLight> CrashPredictor::findTrafficLightById(TrafficLightID id)
{
    return *findTrafficLightIterByID(id);
}

void CrashPredictor::trafficLightReport(const TrafficLightReport &report) {
    std::shared_ptr<TrafficLight> tl;
    try {
        tl = findTrafficLightById(report.id);
    }
    catch (TrafficLightNotFoundException e) {
        tl = std::make_shared<TrafficLight>(report);
        trafficLightList.push_back(tl);
        LOG_I << "Traffic light " << report.id << " add to list.";
    }
    realTimeScene.trafficLightReport(tl, report);
    tl->report(report);
}

bool CrashPredictor::isVehicleRegistered(VehicleID id) {
    try {
        findVehicleById(id);
    }
    catch (VehicleNotFoundException e) {
        return false;
    }
    return true;
}

std::vector<std::shared_ptr<TrafficLight>>::iterator CrashPredictor::findTrafficLightIterByID(TrafficLightID id)
{
    std::lock_guard<std::mutex> lockgrand(trafficLightListMutex);
    std::vector<std::shared_ptr<TrafficLight>>::iterator result = std::find_if(trafficLightList.begin(), trafficLightList.end(), [&](std::shared_ptr<TrafficLight> &item) { return item->getId() == id; });
    if (result == trafficLightList.end()) throw TrafficLightNotFoundException((std::ostringstream() << "Traffic light with id " << id << " not found").str().c_str());
    return result;
}

bool CrashPredictor::isTrafficLightRegistered(TrafficLightID id)
{
    try {
        findTrafficLightIterByID(id);
    }
    catch (TrafficLightNotFoundException e) {
        return false;
    }
    return true;
}


void CrashPredictor::registerVehicle(const VehicleInfo &info) {
    if (isVehicleRegistered(info.id)) {
        LOG_W << "Register vehicle " << info.id << " repeatly.";
        return;
    }
    auto vehicle = std::make_shared<Vehicle>(info);
    {
        std::lock_guard<std::mutex> lockgrand(vehicleListMutex);
        vehicleList.push_back(vehicle);
    }
    LOG_I << "Vehicle " << info.id << " has registered.";

}

void CrashPredictor::unregisterVehicle(VehicleID id) {
    std::vector < std::shared_ptr<Vehicle>>::iterator vehicle;
    try {
        vehicle = findVehicleIterByID(id);
    }
    catch (VehicleNotFoundException e) {
        LOG_W << "Try to unregister vehicle not in vehicleList, which id is " << id;
        return;
    }
    if ((*vehicle)->isBindWithB2Body()) realTimeScene.vehicleUnregister(*vehicle);
    {
        std::lock_guard<std::mutex> lockgrand(vehicleListMutex);
        vehicleList.erase(vehicle);
    }
    LOG_I << "Vehicle " << id << " unregistered.";
}

void CrashPredictor::vehicleReport(const VehicleReport &report) {
    std::shared_ptr<Vehicle> vehicle = nullptr;
    try {
        vehicle = findVehicleById(report.id);
    }
    catch (VehicleNotFoundException e) {
        LOG_W << "Receive report from vehicle " << report.id << ", but vehicle has not registered.";
        throw e;
    }
    if (vehicle == nullptr) {
        LOG_E << "Unexcepted nullptr to vehicle";
        throw Exception("Unexcepted nullptr to vehicle");
    }
    LOG_D << report;
    long long  thisReportTime = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()).time_since_epoch().count();
    if (lastReportTime != 0) {
        // 将实时场景的其他车辆按照回报的时间差推演
        realTimeScene.run(lastReportTime - thisReportTime);
    }
    lastReportTime = thisReportTime;
    vehicle->report(report);
    if (!vehicle->isBindWithB2Body()) {
        realTimeScene.vehicleRegister(vehicle);
    }
    realTimeScene.vehicleReport(vehicle);
    //LOG_D << "Reported";
}

const std::vector<std::shared_ptr<Vehicle>> CrashPredictor::getVehicleList()
{
    return vehicleList;
}

const std::vector<std::shared_ptr<TrafficLight>> CrashPredictor::getTrafficLightList()
{
    return trafficLightList;
}

std::vector<CrashInfo> CrashPredictor::predict(const PredictOption &option)
{
    for (auto &veh : vehicleList) //清除预测缓存
        veh->clearPredictStatusBuf();
    auto scene = std::make_unique<Scene>(this->realTimeScene);

    scene->calculateRecommendVelocity();//计算推荐速度
    auto ret = scene->predict(option);//碰撞预测

    for (auto &veh : vehicleList) //提交缓存的预测
        veh->applyPredictStatus();

    return ret;
}

Scene &CrashPredictor::getRealTimeScene()
{
    return realTimeScene;
}
