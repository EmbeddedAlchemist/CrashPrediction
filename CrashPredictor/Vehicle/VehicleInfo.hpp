#pragma once

#include "VehicleID.hpp"
#include <ostream>


struct VehicleSize {
	float width;  // meters
	float length; // meters
};

using VehicleWeight = float; // kg
using VehicleMaxBrakeForce = float; // N

struct VehicleInfo{
	VehicleID id;
	VehicleSize size;
	VehicleWeight weight;
	VehicleMaxBrakeForce maxBrakeForce;

	friend std::ostream &operator << (std::ostream &out, const VehicleInfo &info);
};

