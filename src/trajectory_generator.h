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
  const double max_speed;
  const double max_acc;

  TrajectoryGenerator(const Map &map, double step_dt, double max_speed, double max_acc);
  ~TrajectoryGenerator(){};

  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t length) const;
  FTrajectory Predict(const Frenet &start, const StatePredictionFunction &predict, size_t length) const;
  CTrajectory FrenetToCartesian(const FTrajectory &trajectory) const;
  size_t TrajectoryLength(double t) const;

 private:
  Coeff MinimizeJerk(const State &start, const State &target, double t) const;
  Coeff Differentiate(const Coeff &coefficients) const;
  double Eval(double x, const Coeff &coefficients) const;
};

}  // namespace PathPlanning

#endif