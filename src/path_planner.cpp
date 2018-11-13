#include "path_planner.h"
#include <set>
#include "utils.h"

PathPlanning::PathPlanner::PathPlanner(const Map &map) : map(map) { this->ego.PrintState(); }

void PathPlanning::PathPlanner::Update(const json &telemetry) {
  this->ParseTelemetry(telemetry);

  ego.PrintState();
}

void PathPlanning::PathPlanner::ParseTelemetry(const json &telemetry) {
  this->ego.Update(telemetry["x"], telemetry["y"], telemetry["s"], telemetry["d"],
                   // Speed from json is MPH
                   Mph2ms(telemetry["speed"]),
                   // Yaw from json is in degrees
                   Deg2rad(telemetry["yaw"]));

  // Previous path data given to the Planner
  this->current_path = {telemetry["previous_path_x"], telemetry["previous_path_y"]};

  // Previous path's end s and d values
  // double end_path_s = telemetry["end_path_s"];
  // double end_path_d = telemetry["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  std::set<int> sensed_vehicles;
  // Reads sensor fusion data and maps it to the current vehicles
  // id, x, y, vx, vy, s, d,
  for (std::vector<double> vehicle_telemetry : sensor_fusion) {
    int id = vehicle_telemetry[0];
    double v_x = vehicle_telemetry[3];
    double v_y = vehicle_telemetry[4];
    double speed = Distance(0, v_y, v_x, 0);
    double yaw = std::atan2(v_y, v_x);

    auto it = this->vehicles.find(id);

    if (it == this->vehicles.end()) {
      Vehicle vehicle = {
          id, vehicle_telemetry[1], vehicle_telemetry[2], vehicle_telemetry[5], vehicle_telemetry[6], speed, yaw};
      this->vehicles.emplace(id, vehicle);
    } else {
      it->second.Update(vehicle_telemetry[1], vehicle_telemetry[2], vehicle_telemetry[5], vehicle_telemetry[6], speed,
                        yaw);
    }
    sensed_vehicles.emplace(id);
  }

  // Removes vehicles that are not sensed anymore
  for (auto it = this->vehicles.cbegin(); it == this->vehicles.cend();) {
    int id = it->first;
    if (sensed_vehicles.find(id) == sensed_vehicles.end()) {
      it = this->vehicles.erase(it);
    } else {
      ++it;
    }
  }
}

PathPlanning::Path PathPlanning::PathPlanner::NextPath() {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  // TODO

  return Path({next_x_vals, next_y_vals});
}
