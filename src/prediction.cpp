#include "prediction.hpp"

#include <constants.hpp>

namespace prediction {

Predictions predictions(const Map &map, const std::vector<Vehicle> &sensor_fusion, double t) {
    Predictions predictions;

    // TODO: constants.hpp
    const double step = 0.5;
    auto time = util::unix_ts();

    for (const Vehicle &car : sensor_fusion) {
        // Calculate keep lane path
        std::vector<Waypoint> trajectory;
        for (size_t i = 0; i < t / 0.5; i++) {
            auto state = car.stateIn(map, i * step);
            trajectory.push_back({time + util::timestamp(i * step * 1000), state});
        }

        // TODO: change lane path(s)

        // Add vehicle trajectories to predictions
        predictions.push_back(
                {
                        car.id(),
                        //TODO constants.hpp
                        4., // Fixed length
                        2., // Fixed width
                        std::vector<Trajectory>{
                                // keep lane trajectory with 1. probability
                                {1., trajectory}
                        }
                }
        );
    }

    return predictions;
}

} // namespace prediction