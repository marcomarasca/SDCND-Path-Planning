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

class TrajectoryGenerator {
 public:
  const double step_dt;

 private:
  const Map &map;
  Coeff MinimizeJerk(const State &start, const State &target, double T) const;
  Coeff Differentiate(const Coeff &coefficients) const;
  double Eval(double x, const Coeff &coefficients) const;

 public:
  TrajectoryGenerator(const Map &map, double step_dt);
  ~TrajectoryGenerator(){};

  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t steps) const;
  FTrajectory Predict(const Frenet &start, const StatePredictionFunction &predict, size_t steps) const;
  CTrajectory FrenetToCartesian(const FTrajectory &trajectory) const;
};

}  // namespace PathPlanning

#endif