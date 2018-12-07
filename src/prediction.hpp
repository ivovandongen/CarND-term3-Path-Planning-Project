#pragma once

#include <coordinates.hpp>
#include <vehicle.hpp>
#include <util/time.hpp>

#include <memory>
#include <vector>

namespace prediction {

struct Waypoint {
    util::timestamp ts;
    Vehicle state;
};

struct Trajectory {
    double probability;
    std::vector<Waypoint> trajectory;
};

struct Prediction {
    int vehicle_id;
    double length;
    double width;
    std::vector<Trajectory> trajectories;
};

using Predictions = std::vector<Prediction>;

/**
 * Take the sensor fusion data and predict rough trajectories with probabilities.
 * @param sensor_fusion the parsed sensor fusion data
 * @param t the timespan for the trajectories
 * @return Predictions
 */
Predictions predictions(const Map &, const std::vector<Vehicle> &sensor_fusion, double t);

} // namespace prediction


