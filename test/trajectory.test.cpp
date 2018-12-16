#include <test.hpp>

#include <constants.hpp>
#include <vehicle.hpp>
#include <map.hpp>
#include <util/time.hpp>

#define private public

#include <trajectory.hpp>

#define public public

#include <fmt/behaviour.hpp>
#include <fmt/collections.hpp>


#include <cmath>
#include <vector>

using namespace trajectory;

Map map() {
    return Map::loadFromFile("../../data/highway_map.csv");
}

TEST(Trajectory, MaxS) {
    const auto ego = VehicleBuilder::newEgoBuilder()
            .withS(MAX_S - 10)
            .withD(LANE_WIDTH / 2.)
            .withV(10)
            .build();

    const auto target = VehicleBuilder::newEgoBuilder()
            .withS(10)
            .withD(LANE_WIDTH / 2.)
            .withV(10)
            .build();

    const behaviour::State targetState{behaviour::Action::KEEP_LANE, target};

    Trajectory previousPath{};

    auto trajectory = trajectory::calculateTrajectory(
            map(),
            ego,
            targetState,
            previousPath
    );

    ASSERT_EQ(trajectory.x.size(), 50);
}
