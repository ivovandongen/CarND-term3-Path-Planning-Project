#include "prediction.hpp"

#include <constants.hpp>

#include <iostream>

namespace prediction {

static double buffer(double v) {
    return 1 * v;
}

Predictions predictions(const Map &map, const Vehicle &ego, const std::vector<Vehicle> &traffic, double t) {
    Predictions predictions{
            ego.lane() != 0,
            true,
            ego.lane() != (NUM_LANES - 1)
    };

    auto ego_projected = ego.stateIn(map, t);
    double buffer_m = buffer(ego.v());

    std::cout << "Buffer: " << buffer_m << std::endl;

    for (auto &car : traffic) {
        auto car_projected = car.stateIn(map, t);

        if (car.lane() == ego.lane()) {
            // Check the way ahead, we're in the same lane
            if ((car.s() > ego.s() && (car_projected.s() - ego_projected.s()) < buffer_m)) {
                // Ahead not free
                predictions.free_ahead = false;

                // Check which car is the closest
                if (predictions.ahead == nullptr
                    || predictions.ahead->s() > car.s()) { // TODO: deal with S wrap-around edge case
                    predictions.ahead = std::make_unique<Vehicle>(car);
                }
            }
        } else if (predictions.free_left
                   && car.lane() - ego.lane() == -1) {
            // Check left
            predictions.free_left = ego.s() - car.s() > 2
                                    || car_projected.s() - ego_projected.s() > buffer_m;
        } else if (predictions.free_right
                   && car.lane() - ego.lane() == 1) {
            // Check right
            predictions.free_right = ego.s() - car.s() > 2
                                     || car_projected.s() - ego_projected.s() > buffer_m;
        }

    }

    return predictions;
}

} // namespace prediction