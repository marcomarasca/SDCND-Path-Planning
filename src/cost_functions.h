#ifndef PP_COST_FUNCTIONS_H
#define PP_COST_FUNCTIONS_H

#include <functional>
#include "utils.h"

namespace PathPlanning {

/**
 * Structure to hold the target lane traffic data such as the lane speed and traffic ahead
 */
struct TrafficData {
  // The number of vehicle ahead of the current vehicle
  size_t lane_traffic;
  // The distance of the closest vehicle ahead
  double min_distance;
  // The average speed of the vehicles ahead of the current vehicle
  double lane_speed;
  // Total number of vehicles detected (ahead and behind in any lane)
  size_t tot_traffic;
};

/**
 * Holds information about the collision for a trajectory
 */
using Collision = std::pair<bool, double>;
using CostFunction = std::function<double(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                                          const TrafficData &traffic_data, double max_speed)>;

namespace CostFunctions {

/**
 * Collision cost: returns 1.0 if a collision will happen anywhere in the trajectory according to the given traffic
 * data, 0.0 otherwise
 */
double CollisionCost(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                     const TrafficData &traffic_data, double max_speed);

/**
 * Buffer cost: Measures the minimum distance to any vehicle along the trajectory, returns a value between [0.0, 1.0]
 */
double BufferCost(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                  const TrafficData &traffic_data, double max_speed);

/**
 * Efficiency cost: Measures the average speed of the of the trajectory, returns a value between [0.0, 1.0]
 */
double AverageSpeedCost(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                        const TrafficData &traffic_data, double max_speed);

/**
 * Lane speed cost: Measures the average speed of the vehicles ahead in the target lane, returns a value between
 * [0.0, 1.0]
 */
double LaneSpeedCost(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                     const TrafficData &traffic_data, double max_speed);

/**
 * Lane traffic cost: Measures the number of vehicles ahead in the target lane, returns a value between
 * [0.0, 1.0]
 */
double LaneTrafficCost(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                       const TrafficData &traffic_data, double max_speed);

/**
 * Change plan cost: Measures the difference between the current plan lane and the target lane, returns a value between
 * [0.0, 1.0]
 */
double ChangePlanCost(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                      const TrafficData &traffic_data, double max_speed);

/**
 * Unfinished plan cost: Measures the difference between the current plan lane and the start lane of the trajectory so
 * that trajectories that didn't reach the current plan yet are penalized, returns a value between [0.0, 1.0]
 */
double UnfinishedPlanCost(const Plan &plan, const Plan &previous_plan, const Collision &collision,
                          const TrafficData &traffic_data, double max_speed);
}  // namespace CostFunctions
}  // namespace PathPlanning

#endif