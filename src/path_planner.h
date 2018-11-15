#ifndef PP_PATH_PLANNER_H
#define PP_PATH_PLANNER_H

#include <map>
#include <vector>

#include "json.hpp"
#include "map.h"
#include "trajectory_generator.h"
#include "vehicle.h"

namespace PathPlanning {

// Total time in seconds for the trajectory
const double TRAJECTORT_T = 1.5;
// Delta t between trajectory points in seconds (same as simulator controller update rate)
const double TRAJECTORY_STEP_DT = 0.02;
// Number of points for the trajectory
const size_t TRAJECTORY_STEPS = TRAJECTORT_T / TRAJECTORY_STEP_DT;
// Max speed in m/s
const double MAX_SPEED = Mph2ms(48);
// Min speed in m/s
const double MIN_SPEED = Mph2ms(15);
// Max acceleration in m/s^2
const double MAX_ACC = 10;
// Ego vehicle radius in meters for surrounding vehicle detection
const double SENSOR_RADIUS = 75;

using json = nlohmann::json;
using Trajectory = std::vector<std::vector<double>>;
using Traffic = std::map<size_t, Vehicle>;
using Predictions = std::map<size_t, FTrajectory>;

class PathPlanner {
 private:
  Map &map;
  Vehicle ego;
  FTrajectory f_trajectory;
  std::vector<Traffic> lanes_traffic;
  Predictions predictions;
  TrajectoryGenerator trajectory_generator;

 public:
  PathPlanner(Map &map);
  ~PathPlanner(){};

  void Update(const json &telemetry);
  Trajectory getGlobalCoordTrajectory();

 private:
  void UpdateEgo(const json &telemetry);
  void UpdateTraffic(const json &telemetry);
  void UpdatePredictions();
  void UpdatePlan();
  void UpdateTrajectory();
  Frenet ComputeTarget(const Frenet &Start, size_t target_lane);
};

}  // namespace PathPlanning

#endif