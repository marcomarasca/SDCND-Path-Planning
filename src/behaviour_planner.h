#ifndef PP_BEHAVIOUR_PLANNER_H
#define PP_BEHAVIOUR_PLANNER_H

#include "trajectory_generator.h"
#include "utils.h"
#include "vehicle.h"

namespace PathPlanning {

// Max speed in m/s
const double MAX_SPEED = Mph2ms(48);
// Min speed in m/s
const double MIN_SPEED = Mph2ms(15);
// Max acceleration in m/s^2
const double MAX_ACC = 10;

using LaneTraffic = std::vector<Vehicle>;
using Traffic = std::vector<LaneTraffic>;

class BehaviourPlanner {
 private:
  static double SafeDistance(double v);
  const TrajectoryGenerator &trajectory_generator;
  Frenet plan;

 public:
  BehaviourPlanner(const TrajectoryGenerator &trajectory_generator);
  ~BehaviourPlanner(){};

  Frenet CurrentPlan();
  void ResetPlan(const Frenet &state);
  FTrajectory UpdatePlan(const Vehicle &ego, const Traffic &traffic, size_t trajectory_steps, double processing_time);

 private:
  std::vector<size_t> AvailableLanes(const Vehicle &vehicle) const;
  double EvaluateTrajectory(const Vehicle &ego, const Traffic &traffic, const FTrajectory &trajectory) const;
  Frenet PredictTarget(const Vehicle &ego, const Traffic &traffic, size_t target_lane, double t) const;
  FTrajectory GenerateTrajectory(const Vehicle &ego, const Frenet &target, size_t trajectory_steps, double processing_time);
  bool VehicleAhead(const Vehicle &ego, const Traffic &traffic, size_t target_lane, Vehicle &ahead) const;
};

}  // namespace PathPlanning

#endif