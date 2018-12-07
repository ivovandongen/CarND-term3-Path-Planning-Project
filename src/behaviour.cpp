#include "behaviour.hpp"

#include <cost.hpp>
#include <constants.hpp>
#include <util/collections.hpp>
#include <util/conversion.hpp>
#include <util/time.hpp>

#include <limits>
#include <map>

namespace behaviour {


Behaviour::Behaviour(const Map &map)
        : map_(map), previous_state_({}) {

}

static double min_buffer(double v) {
    return std::max<>(10., 1 * v);
}

State Behaviour::nextState(const Vehicle &ego, const prediction::Predictions &predictions) {

    if (transitioning(ego)) {
        // Don't change behaviour prematurely
        return previous_state_;
    }

    // Determine next possible actions
    std::vector<Action> possibleSuccessions = successions(ego);

    struct CandidateState {
        Action action;
        std::vector<Vehicle> trajectory;
    };

    std::map<float, CandidateState> costs;
    for (auto &action : possibleSuccessions) {
        // Generate candidate trajectories
        std::vector<Vehicle> trajectory = generateTrajectory(action, ego, predictions);

        if (!trajectory.empty()) {
            float cost = cost::calculateCost(ego, predictions, trajectory);
            costs[cost] = {action, std::move(trajectory)};
        }
    }

    auto &best = costs.begin()->second;
    previous_state_ = State{best.action}
            // TODO: Intialise with trajectory
            .withLane(best.trajectory[1].lane())
            .withSpeed(best.trajectory[1].v());

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
            break;
        case Action::CHANGE_LANE_RIGHT:
            break;
    }
    return trajectory;
}

/**
 * Generate a keep lane trajectory.
 */
Behaviour::Trajectory Behaviour::keepLaneTrajectory(const Vehicle &ego, const prediction::Predictions &predictions) {
    std::vector<Vehicle> trajectory = {ego};
    Behaviour::Kinematics kinematics = getKinematics(ego, predictions);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    // TODO: a
    auto xy = map_.getXY(new_s, ego.d());
    trajectory.emplace_back(ego.id(), xy[0], xy[1], new_s, ego.d(), ego.yaw(), new_v);
    return trajectory;
}


Behaviour::Kinematics Behaviour::getKinematics(const Vehicle &ego, const prediction::Predictions &predictions) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = MAX_ACCELERATION_MS_SQUARED + ego.v();
    double new_position;
    double new_velocity;
    double new_accel;

    auto vehicleAhead = getVehicleAhead(ego, predictions);
    //TODO: determine sensor range
    if (vehicleAhead && vehicleAhead->s() - ego.s() < 40) {
        auto vehicleBehind = getVehicleBehind(ego, predictions);
        if (vehicleBehind) {
            new_velocity = vehicleAhead->v(); //must travel at the speed of traffic, regardless of preferred buffer
        } else {
            double max_velocity_in_front =
                    (vehicleAhead->s() - ego.s() - 2 /**TODO preferred buffer */) + vehicleAhead->v() -
                    0.5 * (0); // TODO: acceleration
            new_velocity = std::min(std::min(max_velocity_in_front, max_velocity_accel_limit),
                                    util::mphToMs(MAX_VELOCITY_MPH));
        }
    } else {
        new_velocity = std::min(max_velocity_accel_limit, util::mphToMs(MAX_VELOCITY_MPH));
    }

    new_accel = new_velocity - ego.v(); //Equation: (v_1 - v_0)/t = acceleration
    new_position = ego.s() + new_velocity + new_accel / 2.0;
    return {new_position, new_velocity, new_accel};

}

tl::optional <Vehicle> Behaviour::getVehicleBehind(const Vehicle &ego, const prediction::Predictions &predictions) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1;
    tl::optional <Vehicle> found{};
    for (const prediction::Prediction &prediction: predictions) {
        auto &vehicle = prediction.trajectories[0].trajectory[0].state;
        if (vehicle.lane() == ego.lane() && vehicle.s() < ego.s() && vehicle.s() > max_s) {
            max_s = vehicle.s();
            found = vehicle;
        }
    }
    return found;
}

tl::optional <Vehicle> Behaviour::getVehicleAhead(const Vehicle &ego, const prediction::Predictions &predictions) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = std::numeric_limits<double>::max();
    tl::optional <Vehicle> found{};
    for (const prediction::Prediction &prediction: predictions) {
        auto &vehicle = prediction.trajectories[0].trajectory[0].state;
        if (vehicle.lane() == ego.lane() && vehicle.s() > ego.s() && vehicle.s() < min_s) {
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
            return previous_state_.lane() != ego.lane();
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
