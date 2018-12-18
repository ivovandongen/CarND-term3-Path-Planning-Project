#include "behaviour.hpp"

#include <cost.hpp>
#include <constants.hpp>
#include <trajectory.hpp>
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
            double diff_s = util::diff_wrapped_abs(vehicleAhead->s(), ego.s(), MAX_S);
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
    double min_s_diff = std::numeric_limits<double>::max();
    tl::optional<Vehicle> found{};
    for (const prediction::Prediction &prediction: predictions) {
        // TODO: Handle alternative trajectories
        auto &vehicle = prediction.trajectories[0].trajectory[0].state;
        auto s_diff = util::diff_wrapped_abs(vehicle.s(), ego.s(), MAX_S);
        if (vehicle.lane() == targetLane && vehicle.isBehindOf(ego) &&  s_diff < min_s_diff) {
            min_s_diff = s_diff;
            found = vehicle;
        }
    }
    return found;
}

tl::optional<Vehicle>
Behaviour::getVehicleAhead(const Vehicle &ego, const prediction::Predictions &predictions, int targetLane) {
    double min_s_diff = std::numeric_limits<double>::max();
    tl::optional<Vehicle> found{};
    for (const prediction::Prediction &prediction: predictions) {
        // TODO: Handle alternative trajectories
        auto &vehicle = prediction.trajectories[0].trajectory[0].state;
        auto s_diff = util::diff_wrapped_abs(vehicle.s(), ego.s(), MAX_S);
        if (vehicle.lane() == targetLane && vehicle.isInFrontOf(ego) && s_diff < min_s_diff) {
            min_s_diff = util::diff_wrapped_abs(vehicle.s(), ego.s(), MAX_S);
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
