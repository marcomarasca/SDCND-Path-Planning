#ifndef PP_TRAJECTORY_GENERATOR_H
#define PP_TRAJECTORY_GENERATOR_H

#include <vector>
#include "utils.h"

namespace PathPlanning {

using Coeff = std::vector<double>;

class TrajectoryGenerator {
 private:
  const double step_dt;
  Coeff MinimizeJerk(const State &start, const State &target, double T) const;
  Coeff Differentiate(const Coeff &coefficients) const;
  double Eval(double x, const Coeff &coefficients) const;

 public:
  TrajectoryGenerator(double step_dt);
  ~TrajectoryGenerator(){};

  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t steps) const;
};

}  // namespace PathPlanning

#endif