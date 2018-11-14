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

const double PATH_STEPS = 50;  // number of steps
const double PATH_DT = 0.02;   // seconds
const double MAX_ACC = 10.0;   // m/s^2

using json = nlohmann::json;

class PathPlanner {
 private:
  Map &map;
  size_t steps;
  Vehicle ego;
  std::map<int, Vehicle> vehicles;
  Trajectory fTrajectory;

  BehaviorPlanner behaviorPlanner;
  TrajectoryGenerator trajectoryGenerator;

 public:
  PathPlanner(Map &map, size_t steps = PATH_STEPS);
  ~PathPlanner(){};

  void Update(const json &telemetry);
  Trajectory getGlobalCoordTrajectory();

 private:
  void UpdateEgo(const json &telemetry);
  void UpdateVehicles(const json &telemetry);
  void UpdateTrajectory();
};

}  // namespace PathPlanning

#endif