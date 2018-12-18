#include <test.hpp>

#include <constants.hpp>

#define private public

#include <vehicle.hpp>

#define public public

#include <cmath>
#include <vector>

TEST(Vehicle, IsInFrontOf) {
    ASSERT_TRUE(VehicleBuilder::newEgoBuilder()
                        .withS(2)
                        .build()
                        .isInFrontOf(
                                VehicleBuilder::newEgoBuilder()
                                        .withS(1)
                                        .build()
                        )
    );

    ASSERT_TRUE(VehicleBuilder::newEgoBuilder()
                        .withS(1)
                        .build()
                        .isInFrontOf(
                                VehicleBuilder::newEgoBuilder()
                                        .withS(MAX_S - 1)
                                        .build()
                        )
    );

    ASSERT_TRUE(VehicleBuilder::newEgoBuilder()
                        .withS(MAX_S)
                        .build()
                        .isInFrontOf(
                                VehicleBuilder::newEgoBuilder()
                                        .withS(MAX_S - 1)
                                        .build()
                        )
    );

    ASSERT_FALSE(VehicleBuilder::newEgoBuilder()
                         .withS(MAX_S - 1)
                         .build()
                         .isInFrontOf(
                                 VehicleBuilder::newEgoBuilder()
                                         .withS(1)
                                         .build()
                         )
    );
}
