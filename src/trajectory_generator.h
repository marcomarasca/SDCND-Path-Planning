#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>

namespace PathPlanning {

struct Frenet {
  std::vector<double> s;
  std::vector<double> d;
};

using FTrajectory = std::vector<Frenet>;

class TrajectoryGenerator {
 private:
  double step_dt;
  std::vector<double> JMT(const std::vector<double> &start, const std::vector<double> &target, double T);

 public:
  TrajectoryGenerator(double step_dt);
  ~TrajectoryGenerator(){};

  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t steps);
};

}  // namespace PathPlanning

#endif