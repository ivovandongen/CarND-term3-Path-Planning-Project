#pragma once

#include <constants.hpp>
#include <map.hpp>
#include <prediction.hpp>
#include <vehicle.hpp>

#include <util/collections.hpp>
#include <util/conversion.hpp>
#include <util/time.hpp>

#include <tl/optional.hpp>

#include <array>

namespace behaviour {

enum class Action {
    INIT,
    KEEP_LANE,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT
};

class State {
public:
    State(Action action, const Vehicle &target)
            : action_(action), target_(target), ts_(util::unix_ts()) {}

    Action action() const {
        return action_;
    };

    const Vehicle &target() const {
        return target_;
    }

    long ts() const {
        return ts_;
    }

private:
    Action action_;
    Vehicle target_;
    long long int ts_;
};

class Behaviour {
public:
    explicit Behaviour(const Map &map);

    virtual ~Behaviour() = default;

    State nextState(const Vehicle &ego, const prediction::Predictions &);

    const State& state() const {
        return previous_state_;
    }

private:

    bool transitioning(const Vehicle &vehicle);

    std::vector<Action> successions(const Vehicle &vehicle);

    using Trajectory = std::vector<Vehicle>;

    Trajectory generateTrajectory(Action state, const Vehicle &ego, const prediction::Predictions &);

    Trajectory keepLaneTrajectory(const Vehicle &ego, const prediction::Predictions &predictions);

    Trajectory changeLaneTrajectory(const Vehicle &ego, Action action, const prediction::Predictions &predictions);

    Trajectory generateRoughTrajectory(const Vehicle &ego, const Vehicle &target, double t, double interval);

    struct Kinematics {
        double s;
        double v;
        double a;
    };

    Behaviour::Kinematics getKinematics(const Vehicle &ego, const prediction::Predictions &predictions, int targetLane, double t = 5);

    tl::optional<Vehicle>
    getVehicleAhead(const Vehicle &ego, const prediction::Predictions &predictions, int targetLane);

    tl::optional<Vehicle>
    getVehicleBehind(const Vehicle &ego, const prediction::Predictions &predictions, int targetLane);

    const Map &map_;
    State previous_state_;

};

} // namespace behaviour
