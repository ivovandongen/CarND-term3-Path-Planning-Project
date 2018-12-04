#include "behaviour.hpp"

#include <util/collections.hpp>
#include <util/conversion.hpp>
#include <util/time.hpp>

#include <iostream>

behaviour::Behaviour::Behaviour(const Map &map)
        : map_(map), previous_state_({}) {

}

static double min_buffer(double v) {
    return std::max<>(10., 1 * v);
}

behaviour::State behaviour::Behaviour::nextState(const Vehicle &ego, const std::vector<Vehicle> &traffic,
                                                 const prediction::Predictions &predictions) {
    if (!transitioning(ego)) {
        std::vector<Action> possibleSuccessions = successions();

//        std::cout << "Succession states: " << possibleSuccessions << std::endl;

//        // TODO: assign cost
        if (util::contains(possibleSuccessions, Action::KEEP_LANE)
            && predictions.free_ahead) {
            // Prefer staying in lane if it is free
            previous_state_ = State{Action::KEEP_LANE}.withLane(ego.lane());
        } else if (util::contains(possibleSuccessions, Action::CHANGE_LANE_LEFT)
                   && predictions.free_left) {
            // If possible, overtake on the left
            previous_state_ = State{Action::CHANGE_LANE_LEFT}.withLane(ego.lane() - 1);
        } else if (util::contains(possibleSuccessions, Action::CHANGE_LANE_RIGHT)
                   && predictions.free_right) {
            // Otherwise...
            previous_state_ = State{Action::CHANGE_LANE_RIGHT}.withLane(ego.lane() + 1);
        } else {
            // Fall back to keeping lane and slowing down
            double distance = predictions.ahead->s() - ego.s();
            double buffer = min_buffer(std::max<>(ego.v(), predictions.ahead->v()));
            previous_state_ = State{Action::KEEP_LANE}
                    .withLane(ego.lane())
                    .withSpeed(
                            std::min(
                                    // If enough distance, max speed / lead car speed
                                    util::mphToMs(MAX_VELOCITY_MPH),
                                    // otherwise backoff speed a bit to increase buffer
                                    distance < buffer
                                    ? predictions.ahead->v() * (distance / buffer)
                                    : predictions.ahead->v()
                            )
                    );
        }
    }

    return previous_state_;
}

bool behaviour::Behaviour::transitioning(const Vehicle &ego) {
    switch (previous_state_.action()) {
        case Action::CHANGE_LANE_LEFT:
        case Action::CHANGE_LANE_RIGHT:
            // Have we reached our target lane?
            return previous_state_.lane() != ego.lane();
        default:
            // Have we spent our time on this state?
            return util::unix_ts() - previous_state_.ts() < 500;
    }
}

std::vector<behaviour::Action> behaviour::Behaviour::successions() {
    switch (previous_state_.action()) {
        case Action::INIT:
            return {Action::KEEP_LANE};
        case Action::KEEP_LANE:
            return {Action::KEEP_LANE, Action::CHANGE_LANE_LEFT, Action::CHANGE_LANE_RIGHT};
        case Action::CHANGE_LANE_LEFT:
            return {Action::KEEP_LANE};
        case Action::CHANGE_LANE_RIGHT:
            return {Action::KEEP_LANE};
    }
}

