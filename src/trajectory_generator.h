#ifndef PP_TRAJECTORY_GENERATOR_H
#define PP_TRAJECTORY_GENERATOR_H

#include <functional>
#include <vector>
#include "map.h"
#include "utils.h"

namespace PathPlanning {

using Coeff = std::vector<double>;
using CTrajectory = std::pair<std::vector<double>, std::vector<double>>;

/**
 * A function that returns the frenet state at the given point in time t
 */
using StatePredictionFunction = std::function<Frenet(double t)>;

/**
 * The class is used to generate trajectories between two frenet states minimizing the jerk
 */
class TrajectoryGenerator {
 private:
  const Map &map;

 public:
  // The delta between time steps of generated trajectories
  const double step_dt;
  const double max_speed;
  const double max_acc;

  TrajectoryGenerator(const Map &map, double step_dt, double max_speed, double max_acc);
  ~TrajectoryGenerator(){};

  /**
   * Generates a trajectory of the given length from the given starting state to the given target state, the trajectory
   * does not include the start state
   */
  FTrajectory Generate(const Frenet &start, const Frenet &target, size_t length) const;

  /**
   * Generates a trajectory of the given length from the given starting state, using the given state prediction
   * function, the trakectory does not include the starting state
   */
  FTrajectory Predict(const Frenet &start, const StatePredictionFunction &predict, size_t length) const;

  /**
   * Converts the given frenet trajectory into a trajectory of cartesian (global) coordinates
   */
  CTrajectory FrenetToCartesian(const FTrajectory &trajectory) const;

  /**
   * Return the length of a trajectory of the given duration (in seconds) using the default step delta t
   */
  size_t TrajectoryLength(double t) const;

 private:
  Coeff MinimizeJerk(const State &start, const State &target, double t) const;
  Coeff Differentiate(const Coeff &coefficients) const;
  double Eval(double x, const Coeff &coefficients) const;
};

}  // namespace PathPlanning

#endif