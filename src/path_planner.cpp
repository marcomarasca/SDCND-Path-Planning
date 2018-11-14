#include "path_planner.h"
#include <set>
#include "utils.h"

PathPlanning::PathPlanner::PathPlanner(Map &map, size_t steps)
    : map(map), steps(steps), behaviorPlanner(map, this->ego), trajectoryGenerator(PATH_DT) {}

void PathPlanning::PathPlanner::Update(const json &telemetry) {
  this->UpdateEgo(telemetry);
  this->UpdateVehicles(telemetry);
  this->UpdateTrajectory();
}

void PathPlanning::PathPlanner::UpdateEgo(const json &telemetry) {
  const std::size_t steps_to_go = telemetry["previous_path_x"].size();
  const std::size_t steps_consumed = this->fTrajectory.size() - steps_to_go;

  double s = telemetry["s"];
  double s_v = 0.0;
  double s_a = 0.0;

  double d = telemetry["d"];
  double d_v = 0.0;
  double d_a = 0.0;

  // If there is a previous trajectory uses the s and d computed previously
  if (steps_to_go > 0 && steps_consumed > 0) {
    size_t trajectoryIdx = steps_consumed - 1;

    s = this->fTrajectory[trajectoryIdx][0];
    d = this->fTrajectory[trajectoryIdx][1];

    // Computes velocity and accelerations components from previous paths
    double prev_s, prev_s_v, prev_d, prev_d_v;

    if (trajectoryIdx == 0) {
      prev_s = this->ego.s;
      prev_d = this->ego.d;
      s_v = (s - prev_s) / PATH_DT;
      d_v = (d - prev_d) / PATH_DT;
      s_a = (s_v - this->ego.s_v) / PATH_DT;
      d_a = (d_v - this->ego.d_v) / PATH_DT;
    } else {
      prev_s = this->fTrajectory[trajectoryIdx - 1][0];
      prev_d = this->fTrajectory[trajectoryIdx - 1][1];
      s_v = (s - prev_s) / PATH_DT;
      d_v = (d - prev_d) / PATH_DT;
      s_a = (s_v - this->ego.s_v) / (steps_consumed * PATH_DT);
      d_a = (d_v - this->ego.d_v) / (steps_consumed * PATH_DT);
    }
  }

  this->ego.UpdatePosition(s, d);
  this->ego.UpdateVelocity(s_v, d_v);
  this->ego.UpdateAcceleration(s_a, d_a);
}

void PathPlanning::PathPlanner::UpdateVehicles(const json &telemetry) {
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  std::set<int> sensed_vehicles;
  // Reads sensor fusion data and maps it to the current vehicles
  // id, x, y, vx, vy, s, d,
  for (std::vector<double> vehicle_telemetry : sensor_fusion) {
    int id = vehicle_telemetry[0];
    double s = vehicle_telemetry[5];
    double d = vehicle_telemetry[6];

    if (d < 0) {
      continue;
    }

    auto it = this->vehicles.find(id);

    if (it == this->vehicles.end()) {
      Vehicle vehicle = {id, s, d};
      this->vehicles.emplace(id, vehicle);
    } else {
      it->second.UpdatePosition(s, d);
      // TODO velocity?
    }

    sensed_vehicles.emplace(id);
  }

  // Removes vehicles that are not sensed anymore
  for (auto it = this->vehicles.cbegin(); it != this->vehicles.cend();) {
    int id = it->first;
    if (sensed_vehicles.find(id) == sensed_vehicles.end()) {
      it = this->vehicles.erase(it);
    } else {
      ++it;
    }
  }
}

void PathPlanning::PathPlanner::UpdateTrajectory() {
  // TODO behavior planning
  PathPlanning::Frenet start = {{this->ego.s, this->ego.s_v, this->ego.s_a},
                                {this->ego.d, this->ego.d_v, this->ego.d_a}};

  PathPlanning::Frenet target = {{this->ego.s + 20, 15, 0.0}, {this->ego.d, 0.0, 0.0}};

  this->fTrajectory = this->trajectoryGenerator.Generate(start, target, steps);
}

PathPlanning::Trajectory PathPlanning::PathPlanner::getGlobalCoordTrajectory() {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  for (auto step : this->fTrajectory) {
    double s = step[0];
    double d = step[1];
    auto coord = this->map.FrenetToCartesian(s, d);

    next_x_vals.emplace_back(coord[0]);
    next_y_vals.emplace_back(coord[1]);
  }

  return Trajectory({next_x_vals, next_y_vals});
}
