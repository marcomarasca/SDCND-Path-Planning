#ifndef PP_TRAJECTORY_GENERATOR_H
#define PP_TRAJECTORY_GENERATOR_H

#include <functional>
#include <vector>
#include "map.h"
#include "utils.h"

namespace PathPlanning {

using Coeff = std::vector<double>;
using CTrajectory = std::pair<std::vector<double>, std::vector<double>>;
using StatePredictionFunction = std::function<Frenet(double t)>;

/**
 * The class is used to generate trajectories between two frenet states minimizing the jerk
 */
class TrajectoryGenerator {
 private:
  const Map &map;

 public:
  // The delta between time steps of generated trajectories
  const double step_dt;

  TrajectoryGenerator(const Map &map, double step_dt);
  ~TrajectoryGenerator(){};

  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t steps) const;
  FTrajectory Predict(const Frenet &start, const StatePredictionFunction &predict, size_t steps) const;
  CTrajectory FrenetToCartesian(const FTrajectory &trajectory) const;

 private:
  Coeff MinimizeJerk(const State &start, const State &target, double T) const;
  Coeff Differentiate(const Coeff &coefficients) const;
  double Eval(double x, const Coeff &coefficients) const;
};

}  // namespace PathPlanning

#endif