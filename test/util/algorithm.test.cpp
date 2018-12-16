#include <test.hpp>

#include <constants.hpp>
#include <util/algorithm.hpp>

#include <vector>

using namespace util;

TEST(Algorithm, diff_wrapped) {
    EXPECT_NEAR(diff_wrapped(MAX_S, 10., MAX_S), 10, .001);
    EXPECT_NEAR(diff_wrapped(10., MAX_S, MAX_S), 10, .001);
    EXPECT_NEAR(diff_wrapped(10., 0., MAX_S), 10, .001);
}
