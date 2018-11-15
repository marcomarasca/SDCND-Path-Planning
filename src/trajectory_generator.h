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
  Coeff JMT(const State &start, const State &target, double T);

 public:
  TrajectoryGenerator(double step_dt);
  ~TrajectoryGenerator(){};

  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t steps);
  FTrajectory Predict(const Frenet &state, size_t steps);
};

}  // namespace PathPlanning

#endif