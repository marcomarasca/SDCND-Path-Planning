#include "trajectory_generator.h"
#include <iostream>
#include "Eigen/Dense"
#include "utils.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

PathPlanning::TrajectoryGenerator::TrajectoryGenerator(double step_dt) : step_dt(step_dt) {}

PathPlanning::FTrajectory PathPlanning::TrajectoryGenerator::Generate(const Frenet &start, const Frenet &target,
                                                                     size_t steps) {
  const double T = steps * step_dt;

  //std::cout << "Generating trajectory for t: " << T << "s (" << steps << " steps)" << std::endl;

  // Computes the trajectory coefficients
  std::vector<double> s_coeff = this->JMT(start.s, target.s, T);
  std::vector<double> d_coeff = this->JMT(start.d, target.d, T);

  PathPlanning::FTrajectory trajectory;

  // Computes the values for each step of the trajectory
  for (size_t i = 1; i <= steps; ++i) {
    const double t = i * step_dt;
    const double t_2 = t * t;
    const double t_3 = t * t_2;
    const double t_4 = t * t_3;
    const double t_5 = t * t_4;

    const double s = s_coeff[0] + s_coeff[1] * t + s_coeff[2] * t_2 + s_coeff[3] * t_3 + s_coeff[4] * t_4 + s_coeff[5] * t_5;
    const double s_v = s_coeff[1] + 2.0 * s_coeff[2] * t + 3 * s_coeff[3] * t_2 + 4 * s_coeff[4] * t_3 + 5 * s_coeff[5] * t_4;
    const double s_a = 2.0 * s_coeff[2] + 6 * s_coeff[3] * t + 12 * s_coeff[4] * t_2 + 20 * s_coeff[5] * t_3;

    const double d = d_coeff[0] + d_coeff[1] * t + d_coeff[2] * t_2 + d_coeff[3] * t_3 + d_coeff[4] * t_4 + d_coeff[5] * t_5;
    const double d_v = d_coeff[1] + 2.0 * d_coeff[2] * t + 3 * d_coeff[3] * t_2 + 4 * d_coeff[4] * t_3 + 5 * d_coeff[5] * t_4;
    const double d_a = 2.0 * d_coeff[2] + 6 * d_coeff[3] * t + 12 * d_coeff[4] * t_2 + 20 * d_coeff[5] * t_3;

    Frenet step = {{s, s_v, s_a}, {d, d_v, d_a}};
    
    trajectory.emplace_back(step);
  }

  //std::cout << "Trajectory:" << std::endl;
  //for (size_t i = 0; i < trajectory.size(); ++i) {
  //  auto s = trajectory[i].s;
  //  std::cout << "Time step " << i << ": s-> " << s[0] << ", s_v->" << s[1] << ", s_a->" << s[2] << std::endl;
  //}

  return trajectory;
}

std::vector<double> PathPlanning::TrajectoryGenerator::JMT(const std::vector<double> &start,
                                                           const std::vector<double> &target, double T) {
  const double T_2 = T * T;
  const double T_3 = T * T_2;
  const double T_4 = T * T_3;
  const double T_5 = T * T_4;

  Matrix3d t_matrix;
  t_matrix <<     T_3,      T_4,      T_5, 
              3 * T_2,  4 * T_3,  5 * T_4,
                6 * T, 12 * T_2, 20 * T_3;

  Vector3d s_vector;
  s_vector << target[0] - (start[0] + start[1] * T + 0.5 * start[2] * T_2), 
              target[1] - (start[1] + start[2] * T),
              target[2] - start[2];

  Vector3d a_vector = t_matrix.inverse() * s_vector;

  return {start[0], start[1], 0.5 * start[2], a_vector(0), a_vector(1), a_vector(2)};
}