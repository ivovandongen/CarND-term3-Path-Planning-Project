#pragma once

#include <behaviour.hpp>
#include <map.hpp>
#include <vehicle.hpp>

#include <vector>

namespace trajectory {

struct Path {
    std::vector<double> x_vals;
    std::vector<double> y_vals;
    double target_v;
};


Path calculatePath(const Map &map, const Vehicle &ego,
                   double ref_vel,
                   const behaviour::State &targetState,
                   const std::vector<double> &previous_path_x,
                   const std::vector<double> &previous_path_y);

}

