#ifndef PP_PATH_PLANNER_H
#define PP_PATH_PLANNER_H

#include <map>
#include <vector>

#include "behavior_planner.h"
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
// Max acceleration in m/s^2
const double MAX_ACC = 10;

using json = nlohmann::json;
using Trajectory = std::vector<std::vector<double>>;

class PathPlanner {
 private:
  Map &map;
  Vehicle ego;
  std::map<int, Vehicle> vehicles;
  FTrajectory f_trajectory;

  BehaviorPlanner behavior_planner;
  TrajectoryGenerator trajectory_generator;

 public:
  PathPlanner(Map &map);
  ~PathPlanner(){};

  void Update(const json &telemetry);
  Trajectory getGlobalCoordTrajectory();

 private:
  void UpdateEgo(const json &telemetry);
  void UpdateVehicles(const json &telemetry);
  void UpdateTrajectory();
  Frenet ComputeTarget(Frenet &Start, int target_lane);
};

}  // namespace PathPlanning

#endif