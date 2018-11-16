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
// Ego vehicle range in meters for vehicles detection (in front and behind)
const double RANGE = 75;
// Min distance to front vehicle, TODO should be paremeterized by velocity
const double SAFE_DISTANCE = VEHICLE_LENGTH * 3;

using json = nlohmann::json;
using Trajectory = std::vector<std::vector<double>>;
using Traffic = std::vector<Vehicle>;

class PathPlanner {
 private:
  const Map &map;
  Vehicle ego;
  FTrajectory f_trajectory;
  std::vector<Traffic> lanes_traffic;
  const TrajectoryGenerator trajectory_generator;

 public:
  PathPlanner(Map &map);
  ~PathPlanner(){};

  void Update(const json &telemetry);
  Trajectory getGlobalCoordTrajectory() const;

 private:
  void UpdateEgo(const json &telemetry);
  void UpdateTraffic(const json &telemetry);
  void UpdatePredictions();
  void UpdatePlan();
  void UpdateTrajectory();

  FTrajectory PredictTrajectory(const Vehicle &vehicle, size_t steps) const;
  Frenet GetTarget(size_t lane, double t) const;
  bool VehicleAhead(size_t lane, Vehicle &vehicle) const;
};

}  // namespace PathPlanning

#endif