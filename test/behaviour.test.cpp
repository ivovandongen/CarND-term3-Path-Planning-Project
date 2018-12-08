#include <test.hpp>

#include <map.hpp>
#include <util/time.hpp>

#define private public

#include <behaviour.hpp>

#define public public

#include <fmt/behaviour.hpp>
#include <fmt/collections.hpp>


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
    ASSERT_TRUE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));

    // Longer than threshold != transitioning
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_FALSE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));

    // Changing lane right and target_lane != current_lane == transitioning
    behaviour.previous_state_ = State(Action::CHANGE_LANE_RIGHT).withLane(1);
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_TRUE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));

    // Changing lane left and target_lane != current_lane == transitioning
    behaviour.previous_state_ = State(Action::CHANGE_LANE_LEFT).withLane(1);
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_TRUE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));
}

TEST(Behaviour, Successions) {
    Map map{{},
            {},
            {},
            {},
            {}};
    Behaviour behaviour{map};

    behaviour.previous_state_ = State{Action::INIT};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().build()), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::CHANGE_LANE_LEFT};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().build()), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::CHANGE_LANE_RIGHT};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().build()), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::KEEP_LANE};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().withD(0 * LANE_WIDTH).build()),
              (std::vector<Action>{Action::KEEP_LANE, Action::CHANGE_LANE_RIGHT}));

    behaviour.previous_state_ = State{Action::KEEP_LANE};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().withD(1 * LANE_WIDTH).build()),
              (std::vector<Action>{Action::KEEP_LANE, Action::CHANGE_LANE_LEFT, Action::CHANGE_LANE_RIGHT}));

    behaviour.previous_state_ = State{Action::KEEP_LANE};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().withD((NUM_LANES - 1) * LANE_WIDTH).build()),
              (std::vector<Action>{Action::KEEP_LANE, Action::CHANGE_LANE_LEFT}));
}

TEST(Behaviour, FMT) {
    ASSERT_EQ("KEEP_LANE", fmt::format("{}", Action::KEEP_LANE));
    ASSERT_EQ("{KEEP_LANE, INIT}", fmt::format("{}", std::vector<Action>{Action::KEEP_LANE, Action::INIT}));

    auto state = State{Action::INIT}
            .withLane(2)
            .withSpeed(util::mphToMs(20));
    state.ts_ = 101;
    ASSERT_EQ("(Action: INIT, time: 101, speed: 20mph, lane: 2)", fmt::format("{}", state));
}