

#pragma once

#include <memory>
#include <vector>

#include "CrashInfo/CrashInfo.hpp"
#include "PredictOption/PredictOption.hpp"
#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_world.h>
#include <CrashPredictor/Vehicle/Vehicle.hpp>
#include <mutex>
#include <CrashPredictor/TrafficLight/TrafficLight.hpp>

/*
* @brief Scene用于容纳模拟场景，实际的物理模拟在这里发生
*/
class Scene{
private:
	std::mutex worldEmulateLock; // 这个互斥锁保证了在进行推算时，其他进程不能修改这个world中的body
	std::unique_ptr<b2World> world;

	void createB2BodyDefByVehicle(const std::shared_ptr<Vehicle> &vehicle, b2BodyDef &def);
	void createB2FixtureDefByVehicle(const std::shared_ptr<Vehicle> &vehicle, b2FixtureDef &def, b2PolygonShape &shape);
	b2Body *createB2BodyByVehicle(const std::shared_ptr<Vehicle> &vehicle);

	void createB2BodyDefByTrafficLight(const std::shared_ptr<TrafficLight> &vehicle, b2BodyDef &def);
	void createB2FixtureDefByTrafficLight(const std::shared_ptr<TrafficLight> &trafficLight, b2FixtureDef &def, b2PolygonShape &shape);
	b2Body *createB2BodyByTrafficLight(const std::shared_ptr<TrafficLight> &trafficLight);


private:
	friend class CrashPredictor;
	void vehicleRegister(std::shared_ptr<Vehicle> &vehicle);
	void vehicleUnregister(std::shared_ptr<Vehicle> &vehicle);
	void vehicleReport(std::shared_ptr<Vehicle> &vehicle, const VehicleReport &report);
	void trafficLightReport(std::shared_ptr<TrafficLight> &trafficLight, const TrafficLightReport &report);

public:

	Scene();
	/*
	* @brief 复制构造，用于从realtime scene复制出来做物理推演
	*/
	Scene(Scene& scene);
	~Scene();

	void run();
	std::vector<CrashInfo> predict(const PredictOption& option);
	void calculateRecommendVelocity();


	b2World *_debugGetWorld();
	
};

