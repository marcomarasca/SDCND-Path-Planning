#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>

#include "behavior_planner.h"
#include "json.hpp"
#include "map.h"
#include "trajectory_generator.h"
#include "vehicle.h"

namespace PathPlanning {

// Number of points for the trajectory
const double TRAJECTORY_STEPS = 100;
// Delta t between trajectory points in seconds
const double TRAJECTORY_STEP_DT = 0.02;
// Total time in seconds for the trajectory
const double TRAJECTORT_T = TRAJECTORY_STEPS * TRAJECTORY_STEP_DT;

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