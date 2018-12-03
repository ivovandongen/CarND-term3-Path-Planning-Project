#include <test.hpp>

#include <conversion.hpp>

#include <vector>

TEST(Conversion, mphToMs) {
    EXPECT_NEAR(mphToMs(1), 0.44704, 0.001);
    EXPECT_NEAR(mphToMs(-100), -44.704, 0.001);
}

TEST(Conversion, msToMph) {
    EXPECT_NEAR(msToMph(1), 2.23694, 0.001);
    EXPECT_NEAR(msToMph(-100), -223.694, 0.001);
}