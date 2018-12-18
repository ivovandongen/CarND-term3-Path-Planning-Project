#include <test.hpp>

#include <constants.hpp>
#include <util/algorithm.hpp>

#include <vector>

using namespace util;

TEST(Algorithm, diff_wrapped_abs) {
    EXPECT_NEAR(diff_wrapped_abs(MAX_S, 10., MAX_S), 10, .001);
    EXPECT_NEAR(diff_wrapped_abs(10., MAX_S, MAX_S), 10, .001);
    EXPECT_NEAR(diff_wrapped_abs(10., 0., MAX_S), 10, .001);
}

TEST(Algorithm, diff_wrapped) {
    EXPECT_NEAR(diff_wrapped(2., 1., 100.), -1, .001);
    EXPECT_NEAR(diff_wrapped(1., 2., 100.), 1, .001);

    EXPECT_NEAR(diff_wrapped(10., 10., 100.), 0, .001);
    EXPECT_NEAR(diff_wrapped(-10., 10., 100.), 20, .001);
    EXPECT_NEAR(diff_wrapped(-10., -10., 100.), -0., .001);

    EXPECT_NEAR(diff_wrapped(1., 99., 100.), -2., .001);
    EXPECT_NEAR(diff_wrapped(99., 1., 100.), 2., .001);
}

TEST(Algorithm, in_front) {
    EXPECT_TRUE(in_front(2., 1., 100.));
    EXPECT_TRUE(in_front(40., 1., 100.));
    EXPECT_FALSE(in_front(1., 2., 100.));
    EXPECT_TRUE(in_front(1., 99., 100.));
    EXPECT_FALSE(in_front(99., 1., 100.));
}
