#pragma once

#include <constants.hpp>
#include <map.hpp>
#include <prediction.hpp>
#include <vehicle.hpp>

#include <util/collections.hpp>
#include <util/conversion.hpp>
#include <util/time.hpp>

#include <iostream>

namespace behaviour {

enum class Action {
    INIT,
    KEEP_LANE,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT
};

class State {
public:
    explicit State(Action action)
            : action_(action), ts_(util::unix_ts()), speed_(util::mphToMs(MAX_VELOCITY_MPH)) {}

    // TODO: Trajectory constraints?
    Action action() const {
        return action_;
    };

    int lane() const {
        return lane_;
    }

    double speed() const {
        return speed_;
    }

    State &withLane(int lane) {
        lane_ = lane;
        return *this;
    }

    State &withSpeed(double speed) {
        speed_ = speed;
        return *this;
    }

    long ts() const {
        return ts_;
    }

    int targetVehicle() const {
        return target_vehicle_;
    }

private:
    Action action_;
    long long int ts_;
    double speed_;

    // Optional
    int lane_ = -1;
    int target_vehicle_ = -1;
};

class Behaviour {
public:
    explicit Behaviour(const Map &map);

    virtual ~Behaviour() = default;

    State nextState(const Vehicle &ego, const std::vector<Vehicle> &traffic, const prediction::Predictions &);

private:

    bool transitioning(const Vehicle &vehicle);

    std::vector<Action> successions();

    const Map &map_;
    State previous_state_;

};


inline std::ostream &operator<<(std::ostream &stream, const Action &action) {
    switch (action) {
        case Action::INIT:
            return stream << "INIT";
        case Action::KEEP_LANE:
            return stream << "KEEP_LANE";;
        case Action::CHANGE_LANE_LEFT:
            return stream << "CHANGE_LANE_LEFT";;
        case Action::CHANGE_LANE_RIGHT:
            return stream << "CHANGE_LANE_RIGHT";;
    }
}

inline std::ostream &operator<<(std::ostream &stream, const State &state) {
    return stream << "(Action:" << state.action()
                  << ", time:" << state.ts()
                  << ", speed:" << util::msToMph(state.speed())
                  << ", lane:" << state.lane()
                  << ", target_vehicle:" << state.targetVehicle()
                  << ")";
}

} // namespace behaviour