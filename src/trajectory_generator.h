#ifndef PP_TRAJECTORY_GENERATOR_H
#define PP_TRAJECTORY_GENERATOR_H

#include <vector>
#include "utils.h"

namespace PathPlanning {

struct Frenet {
  State s;
  State d;

  Frenet(const State &s, const State &d) : s(s), d(d){};
};

using Coeff = std::vector<double>;
using FTrajectory = std::vector<Frenet>;

class TrajectoryGenerator {
 private:
  double step_dt;
  Coeff MinimizeJerk(const State &start, const State &target, double T);
  Coeff Differentiate(const Coeff &coefficients);
  double Eval(double x, const Coeff &coefficients);

 public:
  TrajectoryGenerator(double step_dt);
  ~TrajectoryGenerator(){};

  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t steps);
  FTrajectory Predict(const Frenet &state, size_t steps);
};

}  // namespace PathPlanning

#endif