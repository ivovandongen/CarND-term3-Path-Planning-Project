#pragma once

#include <map.hpp>
#include <vehicle.hpp>

#include <vector>

namespace trajectory {

struct Path {
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    double target_vel;
};


Path calculatePath(const Map &map, const Vehicle &ego,
                   double ref_vel,
                   double target_vel,
                   double target_lane,
                   const std::vector<double> &previous_path_x,
                   const std::vector<double> &previous_path_y);

}

