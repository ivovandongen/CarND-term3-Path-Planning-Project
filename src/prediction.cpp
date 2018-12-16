#include "prediction.hpp"

#include <constants.hpp>

namespace prediction {

Predictions predictions(const Map &map, const std::vector<Vehicle> &sensor_fusion, double t, double interval) {
    Predictions predictions;

    auto time = util::unix_ts();

    for (const Vehicle &car : sensor_fusion) {
        // Calculate keep lane path
        std::vector<Waypoint> trajectory;
        for (size_t i = 0; i < t / 0.5; i++) {
            auto state = car.stateIn(map, i * interval);
            trajectory.push_back({time + util::timestamp(i * interval * 1000), state});
        }

        // TODO: change lane path(s)

        // Add vehicle trajectories to predictions
        predictions.push_back(
                {
                        car.id(),
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