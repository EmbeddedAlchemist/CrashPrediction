#pragma once

#include <memory>
#include <vector>

#include "Scene/Scene.hpp"
#include "Vehicle/Vehicle.hpp"
#include "Vehicle/VehicleReport.hpp"
#include "Vehicle/VehicleInfo.hpp"
#include "Exception/Exception.hpp"
#include <mutex>
#include "TrafficLight/TrafficLight.hpp"
#include "TrafficLight/TrafficLightReport.hpp"

class CrashPredictor{

private:
	Scene realTimeScene;

	std::mutex vehicleListMutex;
	std::vector<std::shared_ptr<Vehicle>> vehicleList;

	std::vector<std::shared_ptr<Vehicle>>::iterator findVehicleIterByID(VehicleID id);
	bool isVehicleRegistered(VehicleID id);

	std::mutex trafficLightListMutex;
	std::vector<std::shared_ptr<TrafficLight>> trafficLightList;

	std::vector<std::shared_ptr<TrafficLight>>::iterator findTrafficLightIterByID(TrafficLightID id);
	bool isTrafficLightRegistered(TrafficLightID id);


public:
	void registerVehicle(const VehicleInfo &info);
	void unregisterVehicle(VehicleID id);
	void vehicleReport(const VehicleReport &report);
	std::shared_ptr<Vehicle> findVehicleById(VehicleID id);
	std::shared_ptr<TrafficLight> findTrafficLightById(TrafficLightID id);

	void trafficLightReport(const TrafficLightReport &report);

	const std::vector<std::shared_ptr<Vehicle>> getVehicleList();
	const std::vector<std::shared_ptr<TrafficLight>> getTrafficLightList();

	std::vector<CrashInfo> predict(const PredictOption &option);

	Scene &getRealTimeScene();
};

