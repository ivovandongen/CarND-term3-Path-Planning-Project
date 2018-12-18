#pragma once

#include <behaviour.hpp>
#include <constants.hpp>
#include <map.hpp>
#include <vehicle.hpp>

#include <vector>

namespace trajectory {

struct Trajectory {
    std::vector<double> x;
    std::vector<double> y;
    double interval = 0.02;
};

/**
 * Calculates a fresh trajectory that is starts out in the heading for the previous path
 * without actually using the previous path points. This ensures smooth transitions whilst
 * maintaining maximum reactiveness to new situations.
 *
 * This also allows for different intervals between points from the previous trajectory and
 * points from the output trajectory. Making this ideal to generate rough trajectories for
 * behaviour planning as well.
 *
 * @param map the map
 * @param ego the current ego state
 * @param targetState the target state
 * @param previousPath the remainder of the previous planned path
 * @param points the number of points int he output trajectory
 * @param interval the interval between points in the output trajectory in seconds
 * @return the new trajectory
 */
Trajectory calculateTrajectory(const Map &map,
                               const Vehicle &ego,
                               const behaviour::State &targetState,
                               const Trajectory &previousPath,
                               size_t points,
                               double interval);

/**
 * Calculates a fresh trajectory that is starts out in the heading for the previous path
 * by using the previous path points. This ensures smooth transitions, but limits the new
 * trajectory to use the same interval between points as the previous interval.
 *
 * @param map the map
 * @param ego the current ego state
 * @param targetState the target state
 * @param previousPath the remainder of the previous planned path
 * @param points the number of points int he output trajectory
  * @return the new trajectory
 */
Trajectory calculateTrajectory(const Map &map,
                               const Vehicle &ego,
                               const behaviour::State &targetState,
                               const Trajectory &previousPath,
                               size_t points = TRAJECTORY_POINTS);

} // namespace trajectory

