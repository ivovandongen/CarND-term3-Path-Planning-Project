#include <test.hpp>

#include <map.hpp>

#include <vector>

TEST(Map, GetFrenetDot) {
    auto map = Map::loadFromFile("../../data/highway_map.csv");

    auto sd_dot = map.getFrenetDot(0, 0, 0, 0);
    ASSERT_EQ(sd_dot[0], 0);
    ASSERT_EQ(sd_dot[1], 0);

    sd_dot = map.getFrenetDot(784.6001, 1135.571, 10, 10);
    ASSERT_FLOAT_EQ(sd_dot[0], 9.76123);
    ASSERT_FLOAT_EQ(sd_dot[1], -10.2332);
}