#include "trajectory_generator.h"
#include "Eigen/Dense"
#include "utils.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

PathPlanning::TrajectoryGenerator::TrajectoryGenerator(double dt): dt(dt) {}

PathPlanning::Trajectory PathPlanning::TrajectoryGenerator::Generate(const Frenet &start, const Frenet &target,
                                                                     size_t steps) {
  const double T = steps * dt;

  // Computes the trajectory coefficients
  std::vector<double> s_coeff = this->JMT(start.s, target.s, T);
  std::vector<double> d_coeff = this->JMT(start.d, target.d, T);

  PathPlanning::Trajectory trajectory;

  // Computes the values for each step of the trajectory
  for (size_t i = 0; i < steps; i++) {
    const double t = i * dt;
    double s = 0;
    double d = 0;
    for (size_t j = 0; j < s_coeff.size(); ++j) {
      double t_p = pow(t, j);
      s += s_coeff[j] * t_p;
      d += d_coeff[j] * t_p;
    }
    std::vector<double> step;
    step.emplace_back(s);
    step.emplace_back(d);
    trajectory.emplace_back(step);
  }

  return trajectory;
}

std::vector<double> PathPlanning::TrajectoryGenerator::JMT(const std::vector<double> &start, const std::vector<double> &target, double T) {
  const double T_2 = T * T;
  const double T_3 = T_2 * T;
  const double T_4 = T_3 * T;
  const double T_5 = T_4 * T;

  Matrix3d t_matrix;
  t_matrix << T_3    , T_4     , T_5, 
              3 * T_2, 4 * T_3 , 5 * T_4,
              6 * T  , 12 * T_2, 20 * T_3;

  Vector3d s_vector;
  s_vector << target[0] - (start[0] + start[1] * T + 0.5 * start[2] * T_2), 
              target[1] - (start[1] + start[2] * T),
              target[2] - start[2];

  Vector3d a_vector = t_matrix.inverse() * s_vector;

  return {start[0], start[1], 0.5 * start[2], a_vector(0), a_vector(1), a_vector(2)};
}