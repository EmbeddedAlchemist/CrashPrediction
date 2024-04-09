#include "VehicleReport.hpp"

std::ostream &operator<<(std::ostream &out, const VehicleReport &report)
{
    return out << "Vehicle " << report.id <<" Report: Speed=" << "todo" << " Position=(" << report.position.x << ',' << report.position.y << ")";
}

VehicleReport::VehicleReport(){
    std::time(&timestamp);
}
