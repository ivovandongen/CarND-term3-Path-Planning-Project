#pragma once

#include <prediction.hpp>
#include <vehicle.hpp>

namespace cost {

float
calculateCost(const Vehicle &ego, const prediction::Predictions &predictions, const std::vector<Vehicle> &trajectory);

}