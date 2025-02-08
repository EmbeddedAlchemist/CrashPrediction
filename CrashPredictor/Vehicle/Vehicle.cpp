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
    // ������ʷ��¼
   // const VehicleReport &lastReport = historyReport.size()<1? *reinterpret_cast<VehicleReport*>(0) : historyReport.back(); //��һ����¼
    std::optional<VehicleReport> lastReport;
    if (historyReport.size() > 0)
        lastReport = historyReport.back();

    historyReport.push(report);
    while (historyReport.size() >= maxHistotyReportNum)
        historyReport.pop();

    //������ٶ�
    if (report.accleration.has_value()) {
        //����ر��˼��ٶȣ��Ǿ�ֱ����
        caculatedAcceleration = report.accleration.value();
    }
    else {
        //����͸������ε��ٶȼ���
        if (not lastReport.has_value())
            return; //û����ʷ��¼��û����
        float timeDiff = ((report.timestamp - lastReport->timestamp) / 1000.f);
        caculatedAcceleration.linear = (report.velocity.linear - lastReport->velocity.linear) / timeDiff;
        //���ǵ��Ƕ�ֻ����0~2pi������������ܻ������Ծ������
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
        throw VehicleHasBindedWithB2BodyException(); //�ظ���
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
