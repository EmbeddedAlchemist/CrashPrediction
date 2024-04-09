#pragma once

#include "VehicleID.hpp"
#include "VehicleInfo.hpp"
#include "VehicleReport.hpp"
#include <box2d/b2_body.h>
#include <queue>
#include <ostream>
#include <optional>

class Vehicle;


struct VehiclePredictContactResult {
	inline VehiclePredictContactResult(VehicleID id, const std::string &&reason) :id(id), reason(reason) {}

	VehicleID id;
	std::string reason; //tailgate_front tailgate_back side opposite

};


struct VehiclePredictStatus {
	std::optional<VehiclePredictContactResult> contact = std::nullopt;
	std::optional<float> recommendSpeed = std::nullopt;
};

class Vehicle {
private:
	const int _type_mark = 1;
private:
	const VehicleInfo info;
	b2Body* b2body = nullptr;
	std::queue<VehicleReport> historyReport;


private:
	friend class Scene;
	friend class CrashPredictor;

	void bindB2Body(b2Body *body);
	b2Body *getB2Body();
	
	VehicleAcceleration caculateAcceleration();

	VehicleReport getLastReport();



public:

	Vehicle(const VehicleInfo &info);
	Vehicle(const Vehicle& obj);
	VehicleID getId() const;
	bool isBindWithB2Body();
	
	void report(const VehicleReport &report);

	VehiclePosition getPosition();
	VehicleVelocity getVelocity();
	VehicleAcceleration getAccleration();

	VehicleSize getSize();
	VehicleWeight getWeight();

	friend std::ostream &operator << (std::ostream &out, const Vehicle &s);

private:
	VehiclePredictStatus predictStatus;
	VehiclePredictStatus predictStatusBuf;

public:
	inline VehiclePredictStatus getPredictStatus() {
		return predictStatus;
	}

	inline void clearPredictStatus() {
		predictStatusBuf.contact = std::nullopt;
		predictStatusBuf.recommendSpeed = std::nullopt;
	}

	inline void commitPredictStatus(const VehiclePredictStatus &status) {
		predictStatusBuf = status;
	}

	inline void applyPredictStatus() { predictStatus = predictStatusBuf; }


	inline VehiclePredictStatus &getPredictStatusBufRef() { return predictStatusBuf; }
};