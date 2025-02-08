#include "VehicleReport.hpp"

std::ostream &operator<<(std::ostream &out, const VehicleReport &report)
{
    return out << "Vehicle " << report.id <<" Report: Speed=" << "todo" << " Position=(" << report.position.x << ',' << report.position.y << ")";
}

VehicleReport::VehicleReport(){
    timestamp = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
}
