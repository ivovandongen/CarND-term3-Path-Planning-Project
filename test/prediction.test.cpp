#include <test.hpp>

#define private public

#include <prediction.hpp>

#define public public

#include <map.hpp>
#include <fmt/prediction.hpp>
#include <fmt/collections.hpp>

#include <cmath>
#include <vector>
#include <string>

using namespace prediction;

TEST(Prediction, TestPredictions) {
    auto map = Map::loadFromFile("../../data/highway_map.csv");

    Predictions expected;
    expected.push_back({1, 4, 4,
                        std::vector<Trajectory>{
                                {
                                        1.0,
                                        std::vector<Waypoint>{
                                                {1001, Vehicle{1, 1., 1., 1., 1., 1., 1.}}
                                        }
                                }
                        }
                       });
    std::vector<Vehicle> sensorFusion{
            Vehicle{1, 784.6001, 1135.571, 0., 0., 0., 20.}
    };
    auto actual = predictions(map, sensorFusion, 5);

    ASSERT_EQ(1, actual.size());
    ASSERT_EQ(1, actual[0].trajectories.size());
    ASSERT_EQ(5 / .5, actual[0].trajectories[0].trajectory.size());
    ASSERT_EQ(1, actual[0].trajectories[0].probability);
}

TEST(Prediction, FMT) {
    Predictions predictions;
    predictions.push_back({1, 4, 4,
                           std::vector<Trajectory>{
                                   {
                                           1.0,
                                           std::vector<Waypoint>{{1001, Vehicle{1, 2., 3., 4., 5., 6., 7.}}}
                                   }
                           }
                          });
    std::string s = fmt::format("{}", predictions);
    ASSERT_EQ(
            "{(Vehicle: 1, size: 4x4, trajectories: {(probability: 1, trajectory: {(ts: 1001, state: (id: 1, s:1, d:4, v:5))})})}",
            s);
}