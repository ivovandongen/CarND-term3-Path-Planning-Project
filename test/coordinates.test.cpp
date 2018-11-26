#include <test.hpp>

#include <coordinates.hpp>

#include <math.h>
#include <vector>

TEST(Cartesian, TestBasic) {
    cartesian::Coordinates coord{1, 2};
    ASSERT_DOUBLE_EQ(coord.x(), 1);
    ASSERT_DOUBLE_EQ(coord.y(), 2);
}

TEST(Cartesian, TestToLocal) {
    const cartesian::Coordinates global{1, 2};
    auto local = global.toLocal(0, 0, 0);
    ASSERT_DOUBLE_EQ(local.x(), 1);
    ASSERT_DOUBLE_EQ(local.y(), 2);

    local = global.toLocal(0, 0, .5 * M_PI);
    ASSERT_DOUBLE_EQ(local.x(), 2);
    ASSERT_DOUBLE_EQ(local.y(), -1);

    local = global.toLocal(0, 0, M_PI);
    ASSERT_DOUBLE_EQ(local.x(), -1);
    ASSERT_DOUBLE_EQ(local.y(), -2);

    local = global.toLocal(1, 1, 0);
    ASSERT_DOUBLE_EQ(local.x(), 0);
    ASSERT_DOUBLE_EQ(local.y(), 1);
}

TEST(Cartesian, TestToGlobal) {
    auto global = cartesian::Coordinates{1, 2}.toGlobal(0, 0, 0);
    ASSERT_DOUBLE_EQ(global.x(), 1);
    ASSERT_DOUBLE_EQ(global.y(), 2);

    global = cartesian::Coordinates{2, -1}.toGlobal(0, 0, .5 * M_PI);
    ASSERT_DOUBLE_EQ(global.x(), 1);
    ASSERT_DOUBLE_EQ(global.y(), 2);

    global = cartesian::Coordinates{-1, -2}.toGlobal(0, 0, M_PI);
    ASSERT_DOUBLE_EQ(global.x(), 1);
    ASSERT_DOUBLE_EQ(global.y(), 2);

    global = cartesian::Coordinates{0, 1}.toGlobal(1, 1, 0);
    ASSERT_DOUBLE_EQ(global.x(), 1);
    ASSERT_DOUBLE_EQ(global.y(), 2);
}
