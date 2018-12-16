#include "behaviour.hpp"

#include <cost.hpp>
#include <constants.hpp>
#include <trigonometry.hpp>
#include <util/algorithm.hpp>
#include <util/collections.hpp>
#include <util/conversion.hpp>
#include <util/time.hpp>

#include <fmt/printf.h>
#include <fmt/behaviour.hpp>
#include <spline.h>

#include <cmath>
#include <limits>
#include <map>

namespace behaviour {

int laneDirection(Action action) {
    switch (action) {
        case Action::CHANGE_LANE_LEFT:
            return -1;
        case Action::CHANGE_LANE_RIGHT:
            return 1;
        default:
            assert(false);
            return 0;
    }
}

Behaviour::Behaviour(const Map &map)
        : map_(map), previous_state_({}, VehicleBuilder::newEgoBuilder().build()) {

}

static double min_buffer(double v) {
    return std::max<>(1., .5 * v);
}

State Behaviour::nextState(const Vehicle &ego, const prediction::Predictions &predictions) {

    // Don't change behaviour prematurely
    if (transitioning(ego)) {
        return previous_state_;
    }

    // Determine next possible actions
    std::vector<Action> possibleSuccessions = successions(ego);

    struct CandidateState {
        Action action;
        std::vector<Vehicle> trajectory;
    };

    std::map<double, CandidateState> costs;
    for (auto &action : possibleSuccessions) {
        // Generate candidate trajectories
        std::vector<Vehicle> trajectory = generateTrajectory(action, ego, predictions);

        if (!trajectory.empty()) {
            double cost = cost::calculateCost(ego, predictions, trajectory);
            costs[cost] = {action, std::move(trajectory)};
        }
    }

    fmt::printf("Costs: ");
    for (auto &cost : costs) {
        fmt::print(fmt::format("{}: {} ", cost.second.action, cost.first));
    }
    fmt::printf("\n");

    auto &best = costs.begin()->second;
    // TODO: Initialise with trajectory
    previous_state_ = State{best.action, best.trajectory[best.trajectory.size() - 1]};

    return previous_state_;
}

std::vector<Vehicle>
Behaviour::generateTrajectory(Action action, const Vehicle &ego, const prediction::Predictions &predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    std::vector<Vehicle> trajectory;
    switch (action) {
        case Action::INIT:
            break;
        case Action::KEEP_LANE:
            trajectory = keepLaneTrajectory(ego, predictions);
            break;
        case Action::CHANGE_LANE_LEFT:
        case Action::CHANGE_LANE_RIGHT:
            trajectory = changeLaneTrajectory(ego, action, predictions);
            break;
    }
    return trajectory;
}

Behaviour::Trajectory Behaviour::generateRoughTrajectory(const Vehicle &ego,
                                                         const Vehicle &target,
//                                                         const trajectory::Path &previousPath,
                                                         double t,
                                                         double interval) {
//    size_t prev_size = previousPath.x.size();
//
//    // Way points
//    std::vector<double> way_pts_x;
//    std::vector<double> way_pts_y;
//
//    // Reference state
//    auto coordinates = ego.coordinates(map_);
//    double ref_x = coordinates.x();
//    double ref_y = coordinates.y();
//    double ref_yaw = deg2rad(ego.yaw());
//
//    double ref_vel = ego.v();
//
//    if (prev_size < 2) {
//        // Not enough previous path points to use as a reference, use
//        // the car position and yaw to get the initial points
//        double prev_car_x = ref_x - cos(ego.yaw());
//        double prev_car_y = ref_y - sin(ego.yaw());
//
//        way_pts_x.push_back(prev_car_x);
//        way_pts_x.push_back(ref_x);
//
//        way_pts_y.push_back(prev_car_y);
//        way_pts_y.push_back(ref_y);
//    } else {
//        // Use the end of the remaining previous path
//        // to start off the next point calculations
//
//        // Limit the number of points used as to limit
//        // reaction time whilst maintaining a smooth
//        // trajectory
//        prev_size = std::min(prev_size, size_t(2));
//
//        // Last point
//        ref_x = previousPath.x[prev_size - 1];
//        ref_y = previousPath.y[prev_size - 1];
//
//        // Point before
//        double ref_x_prev = previousPath.x[prev_size - 2];
//        double ref_y_prev = previousPath.y[prev_size - 2];
//        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
//
//        // Calculate the reference velocity from the last 2 points
//        // (!=ego velocity due to acceleration in the previous path)
//        ref_vel = distance(ref_x_prev, ref_y_prev, ref_x, ref_y) / .02;
//
//        way_pts_x.push_back(ref_x_prev);
//        way_pts_x.push_back(ref_x);
//
//        way_pts_y.push_back(ref_y_prev);
//        way_pts_y.push_back(ref_y);
//    }
//
//    // Add way points
//    // TODO dynamically place waypoints based on distance
//    std::array<int, 3> wp_s{30, 30, 30};
//    for (size_t i = 0; i < wp_s.size(); i++) {
//        auto wp = map_.getXY(ego.s() + ((i + 1) * wp_s[i]), target.d());
//        way_pts_x.push_back(wp[0]);
//        way_pts_y.push_back(wp[1]);
//    }
//
//    assert(way_pts_x.size() == way_pts_y.size());
//
//    // Transform points to local coordinate space
//    for (size_t i = 0; i < way_pts_x.size(); i++) {
//        auto local = cartesian::Coordinates::toLocal(way_pts_x[i], way_pts_y[i], ref_x, ref_y, ref_yaw);
//        way_pts_x[i] = local.x();
//        way_pts_y[i] = local.y();
//    }
//
//    tk::spline spline;
//    spline.set_points(way_pts_x, way_pts_y);
//
//    // Determine target x,y,dist
//    double target_x = 30;
//    double target_y = spline(target_x);
//    double target_dist = sqrt(target_x * target_x + target_y * target_y);
//
//    // Interpolate the spline at set intervals
//    Trajectory trajectory;
//
//    // Add ego as the start point
//    trajectory.push_back(ego);
//
//    double x_add_on = 0;
//    double v = ego.v();
//    int steps = int(t / interval);
//    double max_acc_interval = 5 / interval;
//    for (size_t i = 0; i < steps - 1; i++) {
//        double a = 0;
//        if (ref_vel < target.v()) {
//            a = max_acc_interval;
//        } else if (ref_vel > target.v()) {
//            a = max_acc_interval;
//        }
//
//        ref_vel += a;
//
//        double N = (target_dist / (interval * ref_vel));
//        double x_point = x_add_on + (target_x) / N;
//        double y_point = spline(x_point);
//
//        x_add_on = x_point;
//
//        auto global = cartesian::Coordinates::toGlobal(x_point, y_point, ref_x, ref_y, ref_yaw);
//        auto sd = map_.getFrenet(global.x(), global.y(), ref_yaw);
//
//        trajectory.push_back(
//                VehicleBuilder::newBuilder(ego)
//                        .withA(a)
//                        .withV(v)
//                        .withS(sd[0])
//                        .withD(sd[1])
//                        .build()
//        );
//    }
//
//    // Add target as the endpoint
//    trajectory.push_back(target);
//
//    return trajectory;
    return {};
}

/**
 * Generate a keep lane trajectory.
 */
Behaviour::Trajectory Behaviour::keepLaneTrajectory(const Vehicle &ego, const prediction::Predictions &predictions) {
    Behaviour::Kinematics kinematics = getKinematics(ego, predictions, ego.lane());
    auto target = VehicleBuilder::newBuilder(ego)
            .withS(kinematics.s)
                    // Target center of lane, not current D
            .withD(ego.lane() * LANE_WIDTH + LANE_WIDTH / 2.)
            .withV(kinematics.v)
            .withA(kinematics.a)
            .build();
//    return generateRoughTrajectory(ego, target, 2.5, .5);
    return {ego, target};
}

Behaviour::Trajectory
Behaviour::changeLaneTrajectory(const Vehicle &ego, Action action, const prediction::Predictions &predictions) {
    /*
    Generate a lane change trajectory.
    */
    int targetLane = ego.lane() + laneDirection(action);
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (const prediction::Prediction &prediction:predictions) {
        // TODO: Alternative trajectories?
//        for (const prediction::Waypoint &waypoint : prediction.trajectories[0].trajectory) {
//            // TODO: t
//            const Vehicle &vehicle = waypoint.state;
//            // TODO: overlap
//            if (std::abs(vehicle.s() - ego.s()) < 2 && vehicle.lane() == targetLane) {
//                //If lane change is not possible, return empty trajectory.
//                return trajectory;
//            }
//        }
        // TODO: t
        const Vehicle &vehicle = prediction.trajectories[0].trajectory[0].state;
        // TODO: overlap
        if (std::abs(vehicle.s() - ego.s()) < 2 && vehicle.lane() == targetLane) {
            //If lane change is not possible, return empty trajectory.
            return {};
        }

    }

    Kinematics kinematics = getKinematics(ego, predictions, targetLane);
    auto target = VehicleBuilder::newBuilder(ego)
            .withS(kinematics.s)
            .withD(targetLane * LANE_WIDTH + LANE_WIDTH / 2.)
            .withV(kinematics.v)
            .withA(kinematics.a)
            .build();
//    return generateRoughTrajectory(ego, target, 2.5, .5);
    return {ego, target};
}


Behaviour::Kinematics
Behaviour::getKinematics(const Vehicle &ego, const prediction::Predictions &predictions, int targetLane, double t) {
    /*
    Gets kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = t * MAX_ACCELERATION_MS_SQUARED + ego.v();

    auto vehicleBehindInRange = [&]() {
        auto vehicleBehind = getVehicleBehind(ego, predictions, targetLane);
        return vehicleBehind && ego.s() - vehicleBehind->s() < SENSOR_RANGE;
    };

    auto accelerationFromVelocityDelta = [](const auto &Vi, const auto &Vf, const auto &t) {
        return util::clamp((Vf - Vi) / t, -MAX_ACCELERATION_MS_SQUARED, MAX_ACCELERATION_MS_SQUARED);
    };

    Kinematics k{};

    auto vehicleAhead = getVehicleAhead(ego, predictions, targetLane);
    if (vehicleAhead && vehicleAhead->s() - ego.s() < SENSOR_RANGE) {
        if (vehicleBehindInRange()) {
            // must travel at the speed of traffic, regardless of preferred buffer
            k.v = vehicleAhead->v();
            k.a = accelerationFromVelocityDelta(ego.v(), k.v, t);
        } else {
            double diff_s = vehicleAhead->s() - ego.s();
            double va_distance_travelled = vehicleAhead->v() * t + .5 * vehicleAhead->a() * t * t;
            double Vf = vehicleAhead->v();
            double Vi = ego.v();
            k.a = (Vf * Vf - Vi * Vi) / (2 * (diff_s + va_distance_travelled));
            k.v = ego.v() + k.a * t;
        }
    } else {
        k.v = max_velocity_accel_limit;
        k.a = accelerationFromVelocityDelta(ego.v(), k.v, t);
    }

    k.v = util::clamp(k.v, -MAX_VELOCITY_MS, MAX_VELOCITY_MS);
    k.a = util::clamp(k.a, -MAX_ACCELERATION_MS_SQUARED, MAX_ACCELERATION_MS_SQUARED);
    k.s = std::fmod(ego.s() + (k.v + k.a / 2.0) * t, MAX_S);
    return k;
}

tl::optional<Vehicle>
Behaviour::getVehicleBehind(const Vehicle &ego, const prediction::Predictions &predictions, int targetLane) {
    double max_s = -1;
    tl::optional<Vehicle> found{};
    for (const prediction::Prediction &prediction: predictions) {
        // TODO: Handle alternative trajectories
        auto &vehicle = prediction.trajectories[0].trajectory[0].state;
        if (vehicle.lane() == targetLane && vehicle.s() < ego.s() && vehicle.s() > max_s) {
            max_s = vehicle.s();
            found = vehicle;
        }
    }
    return found;
}

tl::optional<Vehicle>
Behaviour::getVehicleAhead(const Vehicle &ego, const prediction::Predictions &predictions, int targetLane) {
    double min_s = std::numeric_limits<double>::max();
    tl::optional<Vehicle> found{};
    for (const prediction::Prediction &prediction: predictions) {
        // TODO: Handle alternative trajectories
        auto &vehicle = prediction.trajectories[0].trajectory[0].state;
        if (vehicle.lane() == targetLane && vehicle.s() > ego.s() && vehicle.s() < min_s) {
            min_s = vehicle.s();
            found = vehicle;
        }
    }
    return found;
}

bool Behaviour::transitioning(const Vehicle &ego) {
    switch (previous_state_.action()) {
        case Action::INIT:
            return false;
        case Action::CHANGE_LANE_LEFT:
        case Action::CHANGE_LANE_RIGHT:
            // Have we reached our target lane?
            return previous_state_.target().lane() != ego.lane();
        default:
            // Have we spent our time on this state?
            return util::unix_ts() - previous_state_.ts() < 500;
    }
}

std::vector<Action> Behaviour::successions(const Vehicle &ego) {
    switch (previous_state_.action()) {
        case Action::INIT:
            return {Action::KEEP_LANE};
        case Action::CHANGE_LANE_LEFT:
            return {Action::KEEP_LANE};
        case Action::CHANGE_LANE_RIGHT:
            return {Action::KEEP_LANE};
        case Action::KEEP_LANE:
            std::vector<Action> s{Action::KEEP_LANE};
            if (ego.lane() > 0) {
                s.push_back(Action::CHANGE_LANE_LEFT);
            }
            if (ego.lane() < NUM_LANES - 1) {
                s.push_back(Action::CHANGE_LANE_RIGHT);
            }
            return s;
    }
}

} // namespace behaviour
