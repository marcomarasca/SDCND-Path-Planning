#include "trajectory_generator.h"
#include "Eigen/Dense"
#include "utils.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

PathPlanning::TrajectoryGenerator::TrajectoryGenerator(const Map &map, double step_dt) : map(map), step_dt(step_dt) {}

PathPlanning::FTrajectory PathPlanning::TrajectoryGenerator::Generate(const Frenet &start, const Frenet &target,
                                                                      size_t steps) const {
  const double T = steps * step_dt;

  // Computes the trajectory coefficients
  const Coeff s_p_coeff = this->MinimizeJerk(start.s, target.s, T);
  const Coeff s_v_coeff = this->Differentiate(s_p_coeff);
  const Coeff s_a_coeff = this->Differentiate(s_v_coeff);

  const Coeff d_p_coeff = this->MinimizeJerk(start.d, target.d, T);
  const Coeff d_v_coeff = this->Differentiate(d_p_coeff);
  const Coeff d_a_coeff = this->Differentiate(d_v_coeff);

  PathPlanning::FTrajectory trajectory;
  trajectory.reserve(steps);
  // Computes the values for each step of the trajectory
  for (size_t i = 1; i <= steps; ++i) {
    const double t = i * step_dt;

    const double s_p = this->Eval(t, s_p_coeff);
    const double s_v = this->Eval(t, s_v_coeff);
    const double s_a = this->Eval(t, s_a_coeff);

    const double d_p = this->Eval(t, d_p_coeff);
    const double d_v = this->Eval(t, d_v_coeff);
    const double d_a = this->Eval(t, d_a_coeff);

    const State s{s_p, s_v, s_a};
    const State d{d_p, d_v, d_a};

    trajectory.emplace_back(s, d);
  }

  return trajectory;
}

PathPlanning::FTrajectory PathPlanning::TrajectoryGenerator::Predict(const Frenet &start,
                                                                     const StatePredictionFunction &prediction,
                                                                     size_t steps) const {
  FTrajectory trajectory;
  trajectory.reserve(steps);
  for (size_t i = 1; i <= steps; ++i) {
    const double t = i * step_dt;
    trajectory.emplace_back(prediction(t));
  }
  return trajectory;
}

PathPlanning::CTrajectory PathPlanning::TrajectoryGenerator::FrenetToCartesian(const FTrajectory &trajectory) const {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  for (auto &step : trajectory) {
    auto coord = this->map.FrenetToCartesian(step.s.p, step.d.p);
    next_x_vals.emplace_back(coord.first);
    next_y_vals.emplace_back(coord.second);
  }

  return CTrajectory({next_x_vals, next_y_vals});
}

PathPlanning::Coeff PathPlanning::TrajectoryGenerator::Differentiate(const Coeff &coefficients) const {
  Coeff result(coefficients.size() - 1);
  for (size_t i = 1; i < coefficients.size(); ++i) {
    result[i - 1] = i * coefficients[i];
  }
  return result;
}

double PathPlanning::TrajectoryGenerator::Eval(double x, const Coeff &coefficients) const {
  double y = 0;
  for (size_t i = 0; i < coefficients.size(); ++i) {
    y += coefficients[i] * std::pow(x, i);
  }
  return y;
}

PathPlanning::Coeff PathPlanning::TrajectoryGenerator::MinimizeJerk(const State &start,
                                                           const State &target, double T) const {
  const double T_2 = T * T;
  const double T_3 = T * T_2;
  const double T_4 = T * T_3;
  const double T_5 = T * T_4;

  Matrix3d t_matrix;
  t_matrix <<     T_3,      T_4,      T_5, 
              3 * T_2,  4 * T_3,  5 * T_4,
                6 * T, 12 * T_2, 20 * T_3;

  Vector3d s_vector;
  s_vector << target.p - (start.p + start.v * T + 0.5 * start.a * T_2), 
              target.v - (start.v + start.a * T),
              target.a - start.a;

  Vector3d a_vector = t_matrix.inverse() * s_vector;

  return {start.p, start.v, 0.5 * start.a, a_vector(0), a_vector(1), a_vector(2)};
}