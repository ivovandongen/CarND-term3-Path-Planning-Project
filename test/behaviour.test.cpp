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

Map emptyMap() {
    return {{},
            {},
            {},
            {},
            {}};
}

TEST(Behaviour, Transitioning) {
    Behaviour behaviour{emptyMap()};

    // Shorter than threshold == transitioning
    behaviour.previous_state_ = State{Action::KEEP_LANE, VehicleBuilder::newEgoBuilder().build()};
    ASSERT_TRUE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));

    // Longer than threshold != transitioning
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_FALSE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));

    // Changing lane right and target_lane != current_lane == transitioning
    behaviour.previous_state_ = State(Action::CHANGE_LANE_RIGHT,
                                      VehicleBuilder::newEgoBuilder().withD(LANE_WIDTH + .5 * LANE_WIDTH).build());
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_TRUE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));

    // Changing lane left and target_lane != current_lane == transitioning
    behaviour.previous_state_ = State(Action::CHANGE_LANE_LEFT,
                                      VehicleBuilder::newEgoBuilder().withD(LANE_WIDTH + .5 * LANE_WIDTH).build());
    behaviour.previous_state_.ts_ -= 2000;
    ASSERT_TRUE(behaviour.transitioning(VehicleBuilder::newEgoBuilder().build()));
}

TEST(Behaviour, Successions) {
    Behaviour behaviour{emptyMap()};

    behaviour.previous_state_ = State{Action::INIT, VehicleBuilder::newEgoBuilder().build()};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().build()), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::CHANGE_LANE_LEFT, VehicleBuilder::newEgoBuilder().build()};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().build()), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::CHANGE_LANE_RIGHT, VehicleBuilder::newEgoBuilder().build()};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().build()), std::vector<Action>{Action::KEEP_LANE});

    behaviour.previous_state_ = State{Action::KEEP_LANE, VehicleBuilder::newEgoBuilder().build()};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().withD(0 * LANE_WIDTH).build()),
              (std::vector<Action>{Action::KEEP_LANE, Action::CHANGE_LANE_RIGHT}));

    behaviour.previous_state_ = State{Action::KEEP_LANE, VehicleBuilder::newEgoBuilder().build()};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().withD(1 * LANE_WIDTH).build()),
              (std::vector<Action>{Action::KEEP_LANE, Action::CHANGE_LANE_LEFT, Action::CHANGE_LANE_RIGHT}));

    behaviour.previous_state_ = State{Action::KEEP_LANE, VehicleBuilder::newEgoBuilder().build()};
    ASSERT_EQ(behaviour.successions(VehicleBuilder::newEgoBuilder().withD((NUM_LANES - 1) * LANE_WIDTH).build()),
              (std::vector<Action>{Action::KEEP_LANE, Action::CHANGE_LANE_LEFT}));
}

TEST(Behaviour, CarAheadMaxS) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withV(10)
            .withS(MAX_S - 5)
            .withD(LANE_WIDTH / 2.)
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v())
            .withS(5)
            .withD(ego.d())
            .build();

    auto carInFrontOfThat = VehicleBuilder::newBuilder(2)
            .withV(carInFront.v())
            .withS(carInFront.s() + 5)
            .withD(carInFront.d())
            .build();

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }},
            prediction::Prediction{carInFrontOfThat.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFrontOfThat}
                            }
                    }
            }}
    };


    auto vehicleAhead = behaviour.getVehicleAhead(ego, predictions, ego.lane());
    ASSERT_TRUE(vehicleAhead);
    ASSERT_EQ(vehicleAhead->id(), carInFront.id());
}

TEST(Behaviour, CarBehindMaxS) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withV(10)
            .withS(5)
            .withD(LANE_WIDTH / 2.)
            .build();

    auto carBehind = VehicleBuilder::newBuilder(1)
            .withV(ego.v())
            .withS(MAX_S - 5)
            .withD(ego.d())
            .build();

    auto carBehindThat = VehicleBuilder::newBuilder(2)
            .withV(carBehind.v())
            .withS(carBehind.s() - 5)
            .withD(carBehind.d())
            .build();

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carBehind.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carBehind}
                            }
                    }
            }},
            prediction::Prediction{carBehindThat.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carBehindThat}
                            }
                    }
            }}
    };


    auto vehicleBehind = behaviour.getVehicleBehind(ego, predictions, ego.lane());
    ASSERT_TRUE(vehicleBehind);
    ASSERT_EQ(vehicleBehind->id(), carBehind.id());
}

TEST(Behaviour, KinematicsFreeFlow) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withS(0)
            .withD(LANE_WIDTH / 2.)
            .withV(0)
            .build();

    prediction::Predictions predictions{};

    auto kinematics = behaviour.getKinematics(ego, predictions, ego.lane(), 5);
    ASSERT_NEAR(kinematics.s, 120, 1.);
    ASSERT_NEAR(kinematics.v, 22, .1);
    ASSERT_NEAR(kinematics.a, MAX_ACCELERATION_MS_SQUARED, .2);
}

TEST(Behaviour, KinematicsCarAheadNoAcceleration) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withV(10)
            .withS(0)
            .withD(LANE_WIDTH / 2.)
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v())
            .withS(ego.s() + 10)
            .withD(ego.d())
            .build();

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }}
    };

    auto kinematics = behaviour.getKinematics(ego, predictions, ego.lane(), 5);
    ASSERT_NEAR(kinematics.s, 50, 1);
    ASSERT_NEAR(kinematics.v, 10., .1);
    ASSERT_NEAR(kinematics.a, 0., .1);
}

TEST(Behaviour, KinematicsCarAheadAcceleration) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withV(10)
            .withS(0)
            .withD(LANE_WIDTH / 2.)
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v() + 5)
            .withS(ego.s() + 10)
            .withD(ego.d())
            .build();

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }}
    };

    auto kinematics = behaviour.getKinematics(ego, predictions, ego.lane(), 5);
    ASSERT_NEAR(kinematics.s, 70, 1);
    ASSERT_NEAR(kinematics.v, 13.7, .1);
    ASSERT_NEAR(kinematics.a, 0.83, .1);
}

TEST(Behaviour, KinematicsCarAheadDecceleration) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withV(10)
            .withS(0)
            .withD(LANE_WIDTH / 2.)
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v() - 5)
            .withS(ego.s() + 10)
            .withD(ego.d())
            .build();

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }}
    };

    auto kinematics = behaviour.getKinematics(ego, predictions, ego.lane(), 5);
    ASSERT_NEAR(kinematics.s, 20, 1);
    ASSERT_NEAR(kinematics.v, 4.6, .1);
    ASSERT_NEAR(kinematics.a, -1, .1);
}

TEST(Behaviour, KinematicsCarAheadMaxS) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withV(10)
            .withS(MAX_S - 5)
            .withD(LANE_WIDTH / 2.)
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v() - 2)
            .withS(5)
            .withD(ego.d())
            .build();

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }}
    };


    auto kinematics = behaviour.getKinematics(ego, predictions, ego.lane());
    ASSERT_LT(kinematics.v, ego.v());
    ASSERT_GT(kinematics.v, carInFront.v());
    ASSERT_LT(kinematics.a, 0);
}

TEST(Behaviour, KinematicsCarBehindMaxS) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withV(10)
            .withS(MAX_S - 5)
            .withD(LANE_WIDTH / 2.)
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v())
            .withS(5)
            .withD(ego.d())
            .build();

    auto carBehind = VehicleBuilder::newBuilder(2)
            .withV(ego.v())
            .withS(ego.s() - 5)
            .withD(ego.d())
            .build();

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }},
            prediction::Prediction{carBehind.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carBehind}
                            }
                    }
            }}
    };


    auto kinematics = behaviour.getKinematics(ego, predictions, ego.lane());
    ASSERT_EQ(kinematics.v, carInFront.v());
}

TEST(Behaviour, KeepLane) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withD(1 * LANE_WIDTH + LANE_WIDTH / 2.)
            .withV(util::mphToMs(MAX_VELOCITY_MPH))
            .build();

    // Start at keep_lane
    behaviour.previous_state_ = State{Action::KEEP_LANE, ego};
    behaviour.previous_state_.ts_ = 0;

    // Assume we stay at keep lane since we can drive at max vel
    prediction::Predictions predictions;
    auto nextState = behaviour.nextState(ego, predictions);
    ASSERT_EQ(nextState.action(), Action::KEEP_LANE);
}

TEST(Behaviour, ChangeLaneLeft) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withD(2 * LANE_WIDTH + LANE_WIDTH / 2.)
            .withV(util::mphToMs(MAX_VELOCITY_MPH))
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v() - 5)
            .withD(ego.d())
            .withS(ego.s() + 2)
            .build();

    // Start at keep_lane
    behaviour.previous_state_ = State{
            Action::KEEP_LANE,
            VehicleBuilder::newEgoBuilder()
                    .withV(util::mphToMs(MAX_VELOCITY_MPH) / 2)
                    .build()
    };
    behaviour.previous_state_.ts_ = 0;

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }}
    };
    auto nextState = behaviour.nextState(ego, predictions);
    ASSERT_EQ(nextState.action(), Action::CHANGE_LANE_LEFT);

}

TEST(Behaviour, ChangeLaneRight) {
    Behaviour behaviour{emptyMap()};

    auto ego = VehicleBuilder::newEgoBuilder()
            .withD(LANE_WIDTH / 2.)
            .withV(util::mphToMs(MAX_VELOCITY_MPH))
            .build();

    auto carInFront = VehicleBuilder::newBuilder(1)
            .withV(ego.v() - 5)
            .withD(ego.d())
            .withS(ego.s() + 9)
            .build();

    // Start at keep_lane
    behaviour.previous_state_ = State{Action::KEEP_LANE, VehicleBuilder::newEgoBuilder().withV(
            util::mphToMs(MAX_VELOCITY_MPH) / 2).build()};
    behaviour.previous_state_.ts_ = 0;

    // Assume we change lanes as there are no vehicles detected in other lanes
    auto predictions = {
            prediction::Prediction{carInFront.id(), {
                    prediction::Trajectory{
                            1,
                            {
                                    prediction::Waypoint{util::unix_ts(), carInFront}
                            }
                    }
            }}
    };
    auto nextState = behaviour.nextState(ego, predictions);
    ASSERT_EQ(nextState.action(), Action::CHANGE_LANE_RIGHT);

}

TEST(Behaviour, FMT) {
    ASSERT_EQ("KEEP_LANE", fmt::format("{}", Action::KEEP_LANE));
    ASSERT_EQ("{KEEP_LANE, INIT}", fmt::format("{}", std::vector<Action>{Action::KEEP_LANE, Action::INIT}));

    auto state = State{
            Action::INIT,
            VehicleBuilder::newEgoBuilder()
                    .withD(2 * LANE_WIDTH + LANE_WIDTH / 2.)
                    .withV(util::mphToMs(20))
                    .build()
    };

    state.ts_ = 101;
    ASSERT_EQ("(Action: INIT, time: 101, target: (id:-1, lane:2, s:0, d:10, v:8.9408))", fmt::format("{}", state));
}