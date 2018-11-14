#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>

namespace PathPlanning {

using Trajectory = std::vector<std::vector<double>>;

struct Frenet {
  std::vector<double> s;
  std::vector<double> d;
};

class TrajectoryGenerator {
 private:
  double dt;
  std::vector<double> JMT(const std::vector<double> &start, const std::vector<double> &target, double T);

 public:
  TrajectoryGenerator(double dt);
  ~TrajectoryGenerator(){};

  Trajectory Generate(const Frenet &start, const Frenet &target, size_t steps);
};

}  // namespace PathPlanning

#endif