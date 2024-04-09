#include "ContactCallback.h"
#include "ExLog/ExLog.hpp"

#include <ExLog/ExLog.hpp>

void ContactCallback::calcContactResult(std::pair < b2Body *, b2Body *> rec) {
    if (*reinterpret_cast<int *>(rec.first->GetUserData().pointer) != 1 &&
        *reinterpret_cast<int *>(rec.second->GetUserData().pointer) != 1){
        //LOG_D << "not vehicle";
        return;
    }

    if ((rec.first->GetLinearVelocity() - rec.second->GetLinearVelocity()).Length() < option.lowRVIgnoreThreshold) {
        //LOG_D << "low speed";
        return;
    }

    float vehAAngle = rec.first->GetAngle();
    float vehBAngle = rec.second->GetAngle();
    
    float diffAbsAngle = std::min(std::abs(vehAAngle - vehBAngle), std::abs(vehBAngle - vehAAngle));
    Vehicle *vehA = reinterpret_cast<Vehicle *>(rec.first->GetUserData().pointer);
    Vehicle *vehB = reinterpret_cast<Vehicle *>(rec.second->GetUserData().pointer);
    const char *vehAReason = nullptr;
    const char *vehBReason = nullptr;

    LOG_D << vehAAngle << vehBAngle;


    if (diffAbsAngle < 0.7853f) {
        if (rec.first->GetLinearVelocity().LengthSquared() > rec.second->GetLinearVelocity().LengthSquared()) {
            vehAReason = "tailgate_back";
            vehBReason = "tailgate_front";
        }
        else {
            vehBReason = "tailgate_back";
            vehAReason = "tailgate_front";
        }
    }
    else if (diffAbsAngle < 2.3561) {
        vehAReason = vehBReason = "side";
    }
    else {
        vehAReason = vehBReason = "opposite";
    }

    vehA->getPredictStatusBufRef().contact = VehiclePredictContactResult(vehB->getId(), vehAReason);
    vehB->getPredictStatusBufRef().contact = VehiclePredictContactResult(vehA->getId(), vehBReason);


}

void ContactCallback::BeginContact(b2Contact *contact){
    auto rec = std::pair<b2Body *, b2Body *>(contact->GetFixtureA()->GetBody(), contact->GetFixtureB()->GetBody());
    calcContactResult(rec);
    record.push_back(rec);

}

 std::vector<CrashInfo> ContactCallback::getContact()
{
    std::vector<CrashInfo> result = std::vector<CrashInfo>();
    for (std::pair<b2Body *, b2Body *> &rec : record) {

        if (*reinterpret_cast<int *>(rec.first->GetUserData().pointer) != 1 &&
            *reinterpret_cast<int *>(rec.second->GetUserData().pointer) != 1)
            continue;


        CrashInfo info;
        info.vehicleA = reinterpret_cast<Vehicle*>(rec.first->GetUserData().pointer);
        info.vehicleB = reinterpret_cast<Vehicle*>(rec.second->GetUserData().pointer);


        if ((rec.first->GetLinearVelocity() - rec.second->GetLinearVelocity()).Length() < option.lowRVIgnoreThreshold) {
            continue;
        }

        //info.vehicleA->predictStatus.predictedContactVehicle = info.vehicleB;
        //info.vehicleB->predictStatus.predictedContactVehicle = info.vehicleA;
        calcContactResult(rec);

        result.push_back(info);
    }
    return result;
}
