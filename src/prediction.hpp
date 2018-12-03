#pragma once

#include "vehicle.hpp"

#include <ostream>
#include <memory>
#include <vector>

namespace prediction {

struct Predictions {
    bool free_left;
    bool free_ahead;
    bool free_right;

    std::unique_ptr<Vehicle> ahead;
};

Predictions predictions(const Map &, const Vehicle &ego, const std::vector<Vehicle> &traffic, double t);

inline std::ostream &operator<<(std::ostream &stream, const Predictions &p) {
    return stream << "Predictions(left:" << p.free_left << ", ahead:" << p.free_ahead << " ,right:" << p.free_right
                  << ")";
}

} // namespace prediction