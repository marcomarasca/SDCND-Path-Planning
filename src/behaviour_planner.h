#ifndef PP_BEHAVIOUR_PLANNER_H
#define PP_BEHAVIOUR_PLANNER_H

#include "plan_evaluator.h"
#include "trajectory_generator.h"
#include "utils.h"
#include "vehicle.h"

namespace PathPlanning {

// Max speed in m/s
const double MAX_SPEED = Mph2ms(50);
// Min speed in m/s
const double MIN_SPEED = Mph2ms(15);
// Max acceleration in m/s^2
const double MAX_ACC = 10;

class BehaviourPlanner {
  /**
   * Computes a safe distance at the given velocity (e.g. the breaking distance at the max allowed acceleration)
   */
  static double SafeDistance(double v);

 private:
  const TrajectoryGenerator &trajectory_generator;
  const PlanEvaluator plan_evaluator;
  // Current plan
  Plan plan;

 public:
  BehaviourPlanner(const TrajectoryGenerator &trajectory_generator);
  ~BehaviourPlanner(){};

  /**
   * Resets the current plan to the given state
   */
  void ResetPlan(const Frenet &state);

  /**
   * Updates the current plan for the vehicle considering the given traffic and processing time, returns the updated
   * plan (with an empty trajectory if not plan could be generated)
   */
  Plan Update(const Vehicle &ego, const Traffic &traffic, double t, double processing_time);

 private:
  /**
   * Returns the available lanes (indexes) that the given vehicle can switch to
   */
  std::vector<size_t> GetAvailableLanes(const Vehicle &vehicle) const;

  /**
   * Predicts a target frenet state at time t for the given vehicle considering the given traffic and the target lane
   */
  Frenet PredictTarget(const Vehicle &ego, const Traffic &traffic, size_t target_lane, double t) const;

  /**
   * Generates a new candidate plan for the given target lane, accounting for the delay given by the processing time
   */
  Plan GeneratePlan(const Vehicle &ego, const Traffic &traffic, double t, double processing_time, size_t target_lane);

  /**
   * Evaluates the given plan considering the sorrounding traffic
   */
  double EvaluatePlan(const Plan &plan, const Traffic &traffic) const;

  /**
   * Returns true if a vehicle is ahead of the given ego vehicle in the given lane, populates the ahead vehicle with the
   * vehicle in front if found
   */
  bool GetVehicleAhead(const Vehicle &ego, const Traffic &traffic, size_t target_lane, Vehicle &ahead) const;
};

}  // namespace PathPlanning

#endif