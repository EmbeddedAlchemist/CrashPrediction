#include "Vehicle.hpp"
#include <CrashPredictor/Exception/Exception.hpp>
#include <NotNull/NotNull.hpp>
#include <ExLog/ExLog.hpp>

constexpr int maxHistotyReportNum = 5;


Vehicle::Vehicle(const VehicleInfo &info)
	:info(info){
}

Vehicle::Vehicle(const Vehicle& obj)
	:info(obj.info){
}

VehicleID Vehicle::getId() const{
	return info.id;
}

bool Vehicle::isBindWithB2Body()
{
	return b2body!=nullptr;
}

void Vehicle::report(const VehicleReport &report){
	historyReport.push(report);
	while (historyReport.size() >= maxHistotyReportNum) {
		historyReport.pop();
		LOG_V << "old report history removed";
	}
	LOG_V << "Report has add to history of Vehicle " << getId();
}

VehiclePosition Vehicle::getPosition(){
	if (!isBindWithB2Body()) 
		throw VehicleNotBindWithB2BodyException("Vehicle not bind with box2d body.");
	VehiclePosition res;
	auto pos = b2body->GetPosition();
	res.x = pos.x;
	res.y = pos.y;
	res.angle = b2body->GetAngle();
	return res;
	
}

VehicleVelocity Vehicle::getVelocity(){
	if (!isBindWithB2Body())
		throw VehicleNotBindWithB2BodyException("Vehicle not bind with box2d body.");
	VehicleVelocity res;
	res.linear = b2body->GetLinearVelocity().Length();
	res.angular = b2body->GetAngularVelocity();
	return res;
}

VehicleAcceleration Vehicle::getAccleration()
{
	if (!isBindWithB2Body())
		throw VehicleNotBindWithB2BodyException("Vehicle not bind with box2d body.");
	VehicleAcceleration res;
	return res;
}

VehicleSize Vehicle::getSize()
{
	return info.size;
}

VehicleWeight Vehicle::getWeight()
{
	return info.weight;
}

void Vehicle::bindB2Body(b2Body *body){
	NotNull(body);
	if (b2body != nullptr) {
		LOG_E << "Vehicle " << info.id << " are trying to bind with b2Body twice times.";
		throw VehicleHasBindedWithB2BodyException(); //ÖØ¸´°ó¶¨
	}
	b2body = body;
}

b2Body *Vehicle::getB2Body(){
	return b2body;
}

VehicleAcceleration Vehicle::caculateAcceleration()
{
	if (historyReport.size() < 2) throw VehicleRecordNotEnoughException();
	
}

VehicleReport Vehicle::getLastReport()
{
	return historyReport.front();
}

std::ostream &operator<<(std::ostream &out, const Vehicle &s)
{
	out << "Vehicle ID:" << s.getId();
	return out;
}
