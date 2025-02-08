#include "Vehicle.hpp"
#include <CrashPredictor/Exception/Exception.hpp>
#include <NotNull/NotNull.hpp>
#include <ExLog/ExLog.hpp>

constexpr int maxHistotyReportNum = 5;


Vehicle::Vehicle(const VehicleInfo &info)
    :info(info) {
}

Vehicle::Vehicle(const Vehicle &obj)
    :Vehicle(obj.info){
}

VehicleID Vehicle::getId() const {
    return info.id;
}

bool Vehicle::isBindWithB2Body()
{
    return b2body != nullptr;
}

void Vehicle::report(const VehicleReport &report) {
    // 加入历史记录
   // const VehicleReport &lastReport = historyReport.size()<1? *reinterpret_cast<VehicleReport*>(0) : historyReport.back(); //上一条记录
    std::optional<VehicleReport> lastReport;
    if (historyReport.size() > 0)
        lastReport = historyReport.back();

    historyReport.push(report);
    while (historyReport.size() >= maxHistotyReportNum)
        historyReport.pop();

    //计算加速度
    if (report.accleration.has_value()) {
        //如果回报了加速度，那就直接用
        caculatedAcceleration = report.accleration.value();
    }
    else {
        //否则就根据两次的速度计算
        if (not lastReport.has_value())
            return; //没有历史记录，没法算
        float timeDiff = ((report.timestamp - lastReport->timestamp) / 1000.f);
        caculatedAcceleration.linear = (report.velocity.linear - lastReport->velocity.linear) / timeDiff;
        //考虑到角度只能是0~2pi，如果跨零点可能会产生跳跃的现象
        float angleDiff = report.velocity.angular - lastReport->velocity.angular;
        while (angleDiff > b2_pi)
            angleDiff -= 2 * b2_pi;
        while (angleDiff < -b2_pi)
            angleDiff += 2 * b2_pi;
        caculatedAcceleration.angular = angleDiff / timeDiff;
    }
}

VehiclePosition Vehicle::getPosition() {
    if (!isBindWithB2Body())
        throw VehicleNotBindWithB2BodyException("Vehicle not bind with box2d body.");
    VehiclePosition res;
    auto pos = b2body->GetPosition();
    res.x = pos.x;
    res.y = pos.y;
    res.angle = b2body->GetAngle();
    return res;

}

VehicleVelocity Vehicle::getVelocity() {
    if (!isBindWithB2Body())
        throw VehicleNotBindWithB2BodyException("Vehicle not bind with box2d body.");
    VehicleVelocity res;
    res.linear = b2body->GetLinearVelocity().Length();
    res.angular = b2body->GetAngularVelocity();
    return res;
}

VehicleAcceleration Vehicle::getAccleration()
{
    return caculatedAcceleration;
}

VehicleSize Vehicle::getSize()
{
    return info.size;
}

VehicleWeight Vehicle::getWeight()
{
    return info.weight;
}

void Vehicle::bindB2Body(b2Body *body) {
    NotNull(body);
    if (b2body != nullptr) {
        LOG_E << "Vehicle " << info.id << " are trying to bind with b2Body twice times.";
        throw VehicleHasBindedWithB2BodyException(); //重复绑定
    }
    b2body = body;
}

b2Body *Vehicle::getB2Body() {
    return b2body;
}


VehicleReport Vehicle::getLastReport()
{
    return historyReport.back();
}

std::ostream &operator<<(std::ostream &out, const Vehicle &s)
{
    out << "Vehicle ID:" << s.getId();
    return out;
}
