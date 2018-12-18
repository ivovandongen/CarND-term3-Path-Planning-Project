#include "trajectory.hpp"

#include <constants.hpp>
#include <coordinates.hpp>
#include <map.hpp>
#include <trigonometry.hpp>
#include <vehicle.hpp>

#include <spline.h>

#include <array>
#include <cassert>
#include <cmath>
#include <vector>
#include <iostream>

#include <fmt/printf.h>
#include <fmt/format.h>

namespace trajectory {

struct ReferenceState {
    double ref_x;
    double ref_y;
    double ref_yaw;
    double ref_v;
    double prev_ref_x;
    double prev_ref_y;
};


ReferenceState referenceState(const Map &map,
                              const Vehicle &ego,
                              const Trajectory &previousPath,
                              size_t maxPreviousPathSize) {

    size_t prev_size = std::min(maxPreviousPathSize, previousPath.x.size());
    auto coordinates = ego.coordinates(map);

    // Reference state
    ReferenceState state{
            coordinates.x(),
            coordinates.y(),
            deg2rad(ego.yaw()),
            ego.v()
    };

    if (prev_size < 2) {
        // Not enough previous path points to use as a reference, use
        // the car position and yaw to get the initial points
        state.prev_ref_x = state.ref_x - cos(ego.yaw());
        state.prev_ref_y = state.ref_y - sin(ego.yaw());
    } else {
        // Use the end of the remaining previous path
        // to start off the next point calculations

        // Last point
        state.ref_x = previousPath.x[prev_size - 1];
        state.ref_y = previousPath.y[prev_size - 1];

        // Point before
        state.prev_ref_x = previousPath.x[prev_size - 2];
        state.prev_ref_y = previousPath.y[prev_size - 2];
        state.ref_yaw = atan2(state.ref_y - state.prev_ref_y, state.ref_x - state.prev_ref_x);

        // Calculate the reference velocity from the last 2 points
        // (!=ego velocity due to acceleration in the previous path)
        state.ref_v = distance(state.prev_ref_x, state.prev_ref_y, state.ref_x, state.ref_y) / previousPath.interval;
    }

    return state;
}

Trajectory calculateTrajectory(const Map &map,
                               const Vehicle &ego,
                               const behaviour::State &targetState,
                               const Trajectory &previousPath,
                               size_t points,
                               double interval) {

    // Reference state
    ReferenceState refState = referenceState(map, ego, previousPath, 2);

    // Way points
    std::vector<double> way_pts_x;
    std::vector<double> way_pts_y;

    // Add prev state and reference state to shape the beginning of the path
    way_pts_x.push_back(refState.prev_ref_x);
    way_pts_y.push_back(refState.prev_ref_y);

    way_pts_x.push_back(refState.ref_x);
    way_pts_y.push_back(refState.ref_y);

    // Use the ego coordinates/speed as the reference from here on out
    // This makes sure we're not tied to the interval of the previous
    // path for interpolation of the spline
    auto coords = ego.coordinates(map);
    refState.ref_x = coords.x();
    refState.ref_y = coords.y();
    refState.ref_v = ego.v();

    // Add way points
    auto s_diff = targetState.target().s() < ego.s()
                  ? targetState.target().s() + MAX_S - ego.s()
                  : targetState.target().s() - ego.s();
    for (size_t i = 0; i < 3; i++) {
        auto wp = map.getXY(
                ego.s() + (i + 1) * s_diff / 3,
                targetState.target().d());
        way_pts_x.push_back(wp[0]);
        way_pts_y.push_back(wp[1]);
    }

    assert(way_pts_x.size() == way_pts_y.size());

    // Transform points to local coordinate space
    // Using the ego as the reference coordinates for all the points
    for (size_t i = 0; i < way_pts_x.size(); i++) {
        auto local = cartesian::Coordinates::toLocal(way_pts_x[i],
                                                     way_pts_y[i],
                                                     refState.ref_x,
                                                     refState.ref_y,
                                                     refState.ref_yaw);
        way_pts_x[i] = local.x();
        way_pts_y[i] = local.y();
    }

    // Fit a spline
    tk::spline spline;
    spline.set_points(way_pts_x, way_pts_y);

    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
    next_x_vals.reserve(points);
    next_y_vals.reserve(points);

    // Add first point from last iteration
    next_x_vals.push_back(refState.ref_x);
    next_y_vals.push_back(refState.ref_y);

    // Determine target x,y,dist
    double target_x = 30;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    // Interpolate the spline at set intervals
    double x_add_on = 0;
    double a_dot = MAX_ACCELERATION_MS_SQUARED * interval;
    assert(targetState.target().v() <= MAX_VELOCITY_MS);
    for (size_t i = 1; i < points; i++) {
        if (refState.ref_v < targetState.target().v()) {
            refState.ref_v += a_dot; // max 5ms^2 == .1 m /  .02 s
        } else if (refState.ref_v > targetState.target().v()) {
            refState.ref_v -= a_dot;
        }

        // Limit top speed to maximum
        refState.ref_v = std::min(MAX_VELOCITY_MS, refState.ref_v);

        double N = (target_dist / (interval * refState.ref_v));
        double x_point = x_add_on + target_x / N;
        double y_point = spline(x_point);

        x_add_on = x_point;

        auto global = cartesian::Coordinates::toGlobal(x_point,
                                                       y_point,
                                                       refState.ref_x,
                                                       refState.ref_y,
                                                       refState.ref_yaw);
        next_x_vals.push_back(global.x());
        next_y_vals.push_back(global.y());
    }

    assert(next_x_vals.size() == next_y_vals.size());
    assert(next_x_vals.size() == points);

    return {std::move(next_x_vals), std::move(next_y_vals)};
}

Trajectory calculateTrajectory(const Map &map,
                               const Vehicle &ego,
                               const behaviour::State &targetState,
                               const Trajectory &previousPath,
                               size_t points) {

    size_t prev_size = std::min(previousPath.x.size(), size_t(2));

    // Reference state
    ReferenceState refState = referenceState(map, ego, previousPath, prev_size);

    // Way points
    std::vector<double> way_pts_x;
    std::vector<double> way_pts_y;

    // Add prev state and reference state
    way_pts_x.push_back(refState.prev_ref_x);
    way_pts_y.push_back(refState.prev_ref_y);

    way_pts_x.push_back(refState.ref_x);
    way_pts_y.push_back(refState.ref_y);

    // Add way points
    const int wps = 3;
    auto s_diff = targetState.target().s() < ego.s()
                  ? targetState.target().s() + MAX_S - ego.s()
                  : targetState.target().s() - ego.s();
    for (size_t i = 0; i < wps; i++) {
        auto wp = map.getXY(std::fmod(ego.s() + (i + 1) * s_diff / wps, MAX_S),
                            targetState.target().d());
        way_pts_x.push_back(wp[0]);
        way_pts_y.push_back(wp[1]);
    }

    assert(way_pts_x.size() == way_pts_y.size());

    // Transform points to local coordinate space
    for (size_t i = 0; i < way_pts_x.size(); i++) {
        auto local = cartesian::Coordinates::toLocal(way_pts_x[i],
                                                     way_pts_y[i],
                                                     refState.ref_x,
                                                     refState.ref_y,
                                                     refState.ref_yaw);
        way_pts_x[i] = local.x();
        way_pts_y[i] = local.y();
    }

    // Fit a spline to the way points
    tk::spline spline;
    spline.set_points(way_pts_x, way_pts_y);

    // Create the next points
    std::vector<double> next_x_vals;
    std::vector<double> next_y_vals;
    next_x_vals.reserve(points);
    next_y_vals.reserve(points);

    // Add specified number of remaining points from last iteration
    for (size_t i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previousPath.x[i]);
        next_y_vals.push_back(previousPath.y[i]);
    }

    // Determine target x,y,dist
    double target_x = 30;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    // Interpolate the spline at set intervals
    double x_add_on = 0;
    for (size_t i = prev_size; i < points; i++) {
        if (refState.ref_v < targetState.target().v()) {
            refState.ref_v += .1; // max 5ms^2 == .1 m / .02 s
        } else if (refState.ref_v > targetState.target().v()) {
            refState.ref_v -= .1;
        }

        // Make sure we don't exceed the maximum speed
        refState.ref_v = std::min(MAX_VELOCITY_MS, refState.ref_v);

        double N = (target_dist / (.02 * refState.ref_v));
        double x_point = x_add_on + (target_x) / N;
        double y_point = spline(x_point);

        // Validate point
        assert(x_point > x_add_on);
        assert(distance(x_add_on, spline(x_add_on), x_point, y_point) <= MAX_VELOCITY_MS * .02 * 1.05);

        x_add_on = x_point;

        auto global = cartesian::Coordinates::toGlobal(x_point, y_point, refState.ref_x, refState.ref_y,
                                                       refState.ref_yaw);
        next_x_vals.push_back(global.x());
        next_y_vals.push_back(global.y());
    }

    assert(next_x_vals.size() == next_y_vals.size());
    assert(next_x_vals.size() == points);

    return {std::move(next_x_vals), std::move(next_y_vals)};
}

} // namespace trajectory