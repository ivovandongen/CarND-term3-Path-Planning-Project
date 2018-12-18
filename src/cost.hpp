#pragma once

#include <prediction.hpp>
#include <vehicle.hpp>

namespace cost {

double calculateCost(const Vehicle &ego,
                     const prediction::Predictions &predictions,
                     const std::vector<Vehicle> &trajectory);

} // namespace cost