#ifndef PP_TRAJECTORY_EVALUATOR
#define PP_TRAJECTORY_EVALUATOR

#include <vector>
#include "cost_functions.h"
#include "utils.h"

namespace PathPlanning {

class TrajectoryEvaluator {
  const double COLLISION_COST_W = 10000;
  const double UNFINISHED_PLAN_COST_W = 5000;
  const double SPEED_COST_W = 1000;
  const double LANE_TRAFFIC_COST_W = 250;
  const double LANE_SPEED_COST_W = 200;
  const double BUFFER_COST_W = 50;
  const double CHANGE_PLAN_COST_W = 10;

 private:
  const double max_speed;
  const double step_dt;
  const std::vector<std::pair<CostFunction, double>> cost_functions;

 public:
  TrajectoryEvaluator(double max_speed, double step_dt);
  ~TrajectoryEvaluator(){};

  double Evaluate(const FTrajectory &trajectory, const Traffic &traffic, const Frenet &current_plan) const;

 private:
  TrafficData GetTrafficData(const FTrajectory &trajectory, const Traffic &traffic) const;
  Collision DetectCollision(const FTrajectory &trajectory, const Traffic &traffic) const;
  Collision DetectCollision(const FTrajectory &trajectory1, const FTrajectory &trajectory2) const;
};

}  // namespace PathPlanning

#endif