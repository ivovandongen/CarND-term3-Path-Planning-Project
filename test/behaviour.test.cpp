#include <test.hpp>

#include <map.hpp>
#include <util/time.hpp>

#define private public

#include <behaviour.hpp>

#define public public


#include <cmath>
#include <vector>

using namespace behaviour;

TEST(Behaviour, Transitioning) {
    Map map{{},
            {},
            {},
            {},
            {}};
    Behaviour behaviour{map};

    // Shorter than threshold == transitioning
    behaviour.previous_state_ = State{Action::KEEP_LANE};
    ASSERT_TRUE(behaviour.transitioning({Vehicle::EGO_ID, 0, 0, 0, 0, 0, 0}));

    // Longer than threshold != transitioning
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_FALSE(behaviour.transitioning({Vehicle::EGO_ID, 0, 0, 0, 0, 0, 0}));

    // Changing lane right and target_lane != current_lane == transitioning
    behaviour.previous_state_ = State(Action::CHANGE_LANE_RIGHT).withLane(1);
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_TRUE(behaviour.transitioning({Vehicle::EGO_ID, 0, 0, 0, 0, 0, 0}));

    // Changing lane left and target_lane != current_lane == transitioning
    behaviour.previous_state_ = State(Action::CHANGE_LANE_LEFT).withLane(1);
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_TRUE(behaviour.transitioning({Vehicle::EGO_ID, 0, 0, 0, 0, 0, 0}));
}

TEST(Behaviour, Successions) {
    Map map{{},
            {},
            {},
            {},
            {}};
    Behaviour behaviour{map};

    behaviour.previous_state_ = State{Action::INIT};
    ASSERT_EQ(behaviour.successions(), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::KEEP_LANE};
    ASSERT_EQ(behaviour.successions(), (std::vector<Action>{Action::KEEP_LANE, Action::CHANGE_LANE_LEFT, Action::CHANGE_LANE_RIGHT}));

    behaviour.previous_state_ = State{Action::CHANGE_LANE_LEFT};
    ASSERT_EQ(behaviour.successions(), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::CHANGE_LANE_RIGHT};
    ASSERT_EQ(behaviour.successions(), std::vector<Action>{Action::KEEP_LANE});
}