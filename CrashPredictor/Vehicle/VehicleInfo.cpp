#include "VehicleInfo.hpp"

std::ostream &operator<<(std::ostream &out, const VehicleInfo &info)
{
    return out << "Vehicle " << info.id << " Info: Size:" << info.size.width << 'x' << info.size.length << " weight:" << info.weight;
}
