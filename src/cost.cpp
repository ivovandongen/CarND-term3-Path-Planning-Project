#include "cost.hpp"

#include <constants.hpp>
#include <util/conversion.hpp>

#include <cassert>
#include <cmath>
#include <functional>
#include <map>

namespace cost {


/**
 * A function that returns a value between 0 and 1 for x in the
 * range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
 * Useful for cost functions.
 */
double logistic(double x) {
    return 2.0 / (1 + exp(-x)) - 1.0;
}

using CostFunction = std::function<double(const Vehicle &ego,
                                          const prediction::Predictions &predictions,
                                          const std::vector<Vehicle> &trajectory)>;

double targetSpeedCost(const Vehicle &ego, const prediction::Predictions &predictions,
                       const std::vector<Vehicle> &trajectory) {

    // TODO avg velocity
//    double avg_v = trajectory[trajectory.size() - 1].v();
//    double target_v = util::mphToMs(MAX_VELOCITY_MPH); // TODO: is this current vel?
//    return logistic(2 * abs(target_v - avg_v / avg_v));
    double target_v = trajectory[trajectory.size() - 1].v();
    double max_v = MAX_VELOCITY_MS; // TODO: is this current vel?
    return logistic(2 * abs(max_v - target_v));
}

double laneChangeCost(const Vehicle &ego, const prediction::Predictions &predictions,
                      const std::vector<Vehicle> &trajectory) {

    double startD = trajectory[0].d();
    double targetD = trajectory[trajectory.size() - 1].d();
    return logistic(2 * abs(targetD - startD));
}

/**
 * Sum weighted cost functions to get total cost for trajectory.
 */
double calculateCost(const Vehicle &ego,
                     const prediction::Predictions &predictions,
                     const std::vector<Vehicle> &trajectory) {
//    std::map<std::string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    // Add additional cost functions here.
    const static std::vector<CostFunction> costFunctions{
            targetSpeedCost,
            laneChangeCost
    };
    const static std::vector<double> weight_list = {
            10,
            1
    };

    assert(costFunctions.size() == weight_list.size());

    for (size_t i = 0; i < costFunctions.size(); i++) {
        double new_cost = weight_list[i] * costFunctions[i](ego, predictions, trajectory);
        cost += new_cost;
    }

    return cost;
}

} // namespace cost

//float efficiency_cost(traj, target_vehicle, delta, T, predictions){
////Rewards high average speeds.
//s, _, t = traj
//s = to_equation(s)
//avg_v = float(s(t)) / t
//targ_s, _, _, _, _, _ = predictions[target_vehicle].state_in(t)
//targ_v = float(targ_s) / t
//return logistic(2*float(targ_v - avg_v) / avg_v)
//}