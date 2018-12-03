#include "trajectory.hpp"

#include <constants.hpp>
#include <coordinates.hpp>
#include <map.hpp>
#include <trigonometry.hpp>
#include <vehicle.hpp>

#include <spline.h>

#include <array>
#include <cassert>
#include <math.h>
#include <vector>

namespace trajectory {


Path calculatePath(const Map &map, const Vehicle &ego,
                   double ref_vel,
                   double target_vel,
                   double target_lane,
                   const std::vector<double> &previous_path_x,
                   const std::vector<double> &previous_path_y) {

    size_t prev_size = previous_path_x.size();

    // Way points
    std::vector<double> way_pts_x;
    std::vector<double> way_pts_y;

    // Reference state
    double ref_x = ego.x();
    double ref_y = ego.y();
    double ref_yaw = deg2rad(ego.yaw());

    if (prev_size < 2) {
        // Not enough previous path points to use as a reference, use
        // the car position and yaw to get the initial points
        double prev_car_x = ego.x() - cos(ego.yaw());
        double prev_car_y = ego.y() - sin(ego.yaw());

        way_pts_x.push_back(prev_car_x);
        way_pts_x.push_back(ego.x());

        way_pts_y.push_back(prev_car_y);
        way_pts_y.push_back(ego.y());
    } else {
        // Use the end of the remaining previous path
        // to start off the next point calculations

        // Last point
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        // Point before
        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        way_pts_x.push_back(ref_x_prev);
        way_pts_x.push_back(ref_x);

        way_pts_y.push_back(ref_y_prev);
        way_pts_y.push_back(ref_y);
    }

    // Add way points
    std::array<int, 3> wp_intervals{60, 40, 30};
    for (size_t i = 0; i < wp_intervals.size(); i++) {
        auto wp = map.getXY(ego.s() + ((i + 1) * wp_intervals[i]), (2 + 4 * target_lane));
        way_pts_x.push_back(wp[0]);
        way_pts_y.push_back(wp[1]);
    }

    assert(way_pts_x.size() == way_pts_y.size());

    // Transform points to local coordinate space
    for (size_t i = 0; i < way_pts_x.size(); i++) {
        auto local = cartesian::Coordinates::toLocal(way_pts_x[i], way_pts_y[i], ref_x, ref_y, ref_yaw);
        way_pts_x[i] = local.x();
        way_pts_y[i] = local.y();
    }

    tk::spline spline;
    spline.set_points(way_pts_x, way_pts_y);

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
    next_x_vals.reserve(TRAJECTORY_POINTS);
    next_y_vals.reserve(TRAJECTORY_POINTS);

    // Add all remaining points from last iteration
    for (size_t i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    // Determine target x,y,dist
    double target_x = 30;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    // Interpolate the spline at set intervals
    double x_add_on = 0;
    for (size_t i = prev_size; i < TRAJECTORY_POINTS; i++) {
        if (ref_vel < target_vel) {
            ref_vel += .1; // max 5ms^2 == .1 m / .02 s
        } else if (ref_vel > target_vel) {
            ref_vel -= .1;
        }

        double N = (target_dist / (.02 * ref_vel));
        double x_point = x_add_on + (target_x) / N;
        double y_point = spline(x_point);

        x_add_on = x_point;

        auto global = cartesian::Coordinates::toGlobal(x_point, y_point, ref_x, ref_y, ref_yaw);
        next_x_vals.push_back(global.x());
        next_y_vals.push_back(global.y());
    }

    assert(next_x_vals.size() == next_y_vals.size());
    assert(next_x_vals.size() == TRAJECTORY_POINTS);

    return {std::move(next_x_vals), std::move(next_y_vals), ref_vel};
}

} // namespace trajectory