#ifndef PP_COST_FUNCTIONS_H
#define PP_COST_FUNCTIONS_H

#include <functional>
#include "utils.h"

namespace PathPlanning {

/**
 * Structure to hold the target lane traffic data such as the lane speed and traffic ahead
 */
struct TrafficData {
  size_t target_lane;
  size_t lane_traffic;
  double lane_speed;
  size_t tot_traffic;
};

/**
 * Holds information about the collision for a trajectory
 */
using Collision = std::pair<bool, double>;
using CostFunction =
    std::function<double(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                         const TrafficData &traffic_data, double max_speed, double step_dt)>;

namespace CostFunctions {

double CollisionCost(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                     const TrafficData &traffic_data, double max_speed, double step_dt);
double BufferCost(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                  const TrafficData &traffic_data, double max_speed, double step_dt);
double AverageSpeedCost(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                        const TrafficData &traffic_data, double max_speed, double step_dt);
double LaneSpeedCost(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                     const TrafficData &traffic_data, double max_speed, double step_dt);
double LaneTrafficCost(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                       const TrafficData &traffic_data, double max_speed, double step_dt);
double ChangePlanCost(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                      const TrafficData &traffic_data, double max_speed, double step_dt);
double UnfinishedPlanCost(const FTrajectory &trajectory, const Frenet &current_plan, const Collision &collision,
                          const TrafficData &traffic_data, double max_speed, double step_dt);
}  // namespace CostFunctions
}  // namespace PathPlanning

#endif