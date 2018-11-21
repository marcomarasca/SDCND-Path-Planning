#ifndef PP_PLAN_EVALUATOR_H
#define PP_PLAN_EVALUATOR_H

#include <vector>
#include "cost_functions.h"
#include "utils.h"

namespace PathPlanning {

/**
 * Used to evaluate a given plan generated from the behaviour planner
 */
class PlanEvaluator {
  // Cost weights
  const double COLLISION_COST_W = 50000;
  const double UNFINISHED_PLAN_COST_W = 10000;
  const double SPEED_COST_W = 1000;
  const double LANE_TRAFFIC_COST_W = 250;
  const double LANE_SPEED_COST_W = 200;
  const double BUFFER_COST_W = 50;
  const double CHANGE_PLAN_COST_W = 20;

 private:
  const double max_speed;
  const std::vector<std::pair<CostFunction, double>> cost_functions;

 public:
  PlanEvaluator(double max_speed);
  ~PlanEvaluator(){};

  /**
   * Evaluates the given plan and returns its cost
   */
  double Evaluate(const Plan &plan, const Plan &previous_plan, const Traffic &traffic) const;

 private:
  /**
   * Generates the traffic data for the target lane of the given trajectory using the given traffic information
   */
  TrafficData GetTrafficData(const FTrajectory &trajectory, const Traffic &traffic) const;
  /**
   * Generates collision data for the given trajectory comparing to the predictions stored in the traffic data
   */
  Collision DetectCollision(const FTrajectory &trajectory, const Traffic &traffic) const;
  /**
   * Generates collision data between the two trajectories
   */
  Collision DetectCollision(const FTrajectory &trajectory1, const FTrajectory &trajectory2) const;
};

}  // namespace PathPlanning

#endif