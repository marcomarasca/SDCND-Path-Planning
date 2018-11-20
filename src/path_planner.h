#ifndef PP_PATH_PLANNER_H
#define PP_PATH_PLANNER_H

#include <array>
#include <map>
#include <vector>

#include "json.hpp"

#include "behaviour_planner.h"
#include "map.h"
#include "trajectory_generator.h"
#include "vehicle.h"

namespace PathPlanning {

// Id of the ego vehicle
const int EGO_ID = -1;
// Ego vehicle range in meters for vehicles detection (in front and behind)
const double RANGE = 80;
// Total time in seconds for the trajectory
const double TRAJECTORY_T = 2;
// Delta t between trajectory points in seconds (same as simulator controller update rate)
const double TRAJECTORY_STEP_DT = 0.02;
// Number of points for the trajectory
const size_t TRAJECTORY_STEPS = TRAJECTORY_T / TRAJECTORY_STEP_DT;
// Min number of second for the processing time to consider when including points from the past trajectory
const double MIN_PROCESSING_TIME = 5 * TRAJECTORY_STEP_DT;

using json = nlohmann::json;

class PathPlanner {
 private:
  const Map &map;
  Vehicle ego;
  Traffic traffic;
  TrajectoryGenerator trajectory_generator;
  BehaviourPlanner behaviour_planner;

 public:
  PathPlanner(Map &map, size_t lane_n);
  ~PathPlanner(){};

  void Update(const json &telemetry, double processing_time);
  CTrajectory GetTrajectory() const;

 private:
  void UpdateEgo(const json &telemetry);
  void UpdateTraffic(const json &telemetry);
  void UpdatePredictions();
  void UpdatePlan(double processing_time);
};

}  // namespace PathPlanning

#endif