#include "trajectory_generator.h"
#include "Eigen/Dense"
#include "utils.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

PathPlanning::TrajectoryGenerator::TrajectoryGenerator(const Map &map, double step_dt, double max_speed, double max_acc)
    : map(map), step_dt(step_dt), max_speed(max_speed), max_acc(max_acc) {}

PathPlanning::FTrajectory PathPlanning::TrajectoryGenerator::Generate(const Frenet &start, const Frenet &target,
                                                                      size_t length) const {
  const double t = length * this->step_dt;
  // Computes the trajectory coefficients
  const Coeff s_p_coeff = this->MinimizeJerk(start.s, target.s, t);
  const Coeff s_v_coeff = this->Differentiate(s_p_coeff);
  const Coeff s_a_coeff = this->Differentiate(s_v_coeff);

  const Coeff d_p_coeff = this->MinimizeJerk(start.d, target.d, t);
  const Coeff d_v_coeff = this->Differentiate(d_p_coeff);
  const Coeff d_a_coeff = this->Differentiate(d_v_coeff);

  PathPlanning::FTrajectory trajectory;
  trajectory.reserve(length);
  Frenet prev_state = start;

  for (size_t i = 1; i <= length; ++i) {
    const double t = i * this->step_dt;

    const double max_s_delta = prev_state.s.v * this->step_dt + 0.5 * prev_state.s.a * this->step_dt * this->step_dt;

    // Reduces longitudinal values to meet speed and acceleration constraints
    const double s_p = std::min(this->Eval(t, s_p_coeff), prev_state.s.p + max_s_delta);
    const double s_v = std::min(this->Eval(t, s_v_coeff), this->max_speed);
    const double s_a = std::max(std::min(this->Eval(t, s_a_coeff), this->max_acc), -this->max_acc);

    const double d_p = this->Eval(t, d_p_coeff);
    const double d_v = this->Eval(t, d_v_coeff);
    const double d_a = this->Eval(t, d_a_coeff);

    const State s{Map::Mod(s_p), s_v, s_a};
    const State d{d_p, d_v, d_a};

    trajectory.emplace_back(s, d);
    prev_state = {s, d};
  }

  return trajectory;
}

PathPlanning::FTrajectory PathPlanning::TrajectoryGenerator::Predict(const Frenet &start,
                                                                     const StatePredictionFunction &prediction,
                                                                     size_t length) const {
  FTrajectory trajectory;
  trajectory.reserve(length);
  for (size_t i = 1; i <= length; ++i) {
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

size_t PathPlanning::TrajectoryGenerator::TrajectoryLength(double t) const { return t / this->step_dt; }

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

PathPlanning::Coeff PathPlanning::TrajectoryGenerator::MinimizeJerk(const State &start, const State &target,
                                                                    double t) const {
  const double t_2 = t * t;
  const double t_3 = t * t_2;
  const double t_4 = t * t_3;
  const double t_5 = t * t_4;

  Matrix3d t_matrix;
  t_matrix <<     t_3,      t_4,      t_5, 
              3 * t_2,  4 * t_3,  5 * t_4,
                6 * t, 12 * t_2, 20 * t_3;

  Vector3d s_vector;
  s_vector << target.p - (start.p + start.v * t + 0.5 * start.a * t_2), 
              target.v - (start.v + start.a * t),
              target.a - start.a;

  Vector3d a_vector = t_matrix.inverse() * s_vector;

  return {start.p, start.v, 0.5 * start.a, a_vector(0), a_vector(1), a_vector(2)};
}