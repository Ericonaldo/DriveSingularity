#pragma once

#include <cstddef>
#include <limits>

namespace ds {

using VehicleId = std::size_t;
const VehicleId ErrorVehicleId = std::numeric_limits<VehicleId>::max();

} // namespace ds

