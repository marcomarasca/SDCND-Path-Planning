#ifndef PP_TRAJECTORY_EVALUATOR
#define PP_TRAJECTORY_EVALUATOR

#include "utils.h"

namespace PathPlanning {

class TrajectoryEvaluator {
 private:
  double max_speed;
  double step_dt;

 public:
  TrajectoryEvaluator(double max_speed, double step_dt);
  ~TrajectoryEvaluator(){};

  double Evaluate(const FTrajectory &trajectory, const Traffic &traffic, const Frenet &current_plan) const;
};

}  // namespace PathPlanning

#endif