#include "path_planner.h"
#include <algorithm>
#include <set>
#include "utils.h"

PathPlanning::PathPlanner::PathPlanner(Map &map)
    : map(map), lanes_traffic(LANES_N), trajectory_generator(TRAJECTORY_STEP_DT) {}

void PathPlanning::PathPlanner::Update(const json &telemetry) {
  this->UpdateEgo(telemetry);
  this->UpdateTraffic(telemetry);
  this->UpdatePredictions();
  this->UpdatePlan();
  this->UpdateTrajectory();
}

void PathPlanning::PathPlanner::UpdateEgo(const json &telemetry) {
  const std::size_t steps_to_go = telemetry["previous_path_x"].size();
  const std::size_t steps_consumed = this->f_trajectory.size() - steps_to_go;

  const double s_p = Map::WrapDistance(telemetry["s"]);
  const double s_v = Mph2ms(telemetry["speed"]);
  const double d_p = telemetry["d"];

  Frenet state{{s_p, s_v, 0.0}, {d_p, 0.0, 0.0}};

  std::cout << "Prev Path: " << steps_to_go << std::endl;
  std::cout << "Steps Consumed: " << steps_consumed << std::endl;
  std::cout << "Telemetry: (S: " << state.s.p << ", D: " << state.d.p << ", Speed: " << state.s.v << ")" << std::endl;

  if (steps_to_go == 0) {  // Empty path or all points consumed
    this->f_trajectory.clear();
  } else if (steps_consumed > 0) {  // Use data from the previous trajectory
    state = this->f_trajectory[steps_consumed - 1];
  }

  this->ego.UpdateState(state);
}

void PathPlanning::PathPlanner::UpdateTraffic(const json &telemetry) {
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  std::vector<Traffic> lanes_traffic(LANES_N);

  // Reads sensor fusion data and maps it to the current vehicles
  // id, x, y, vx, vy, s, d,
  for (std::vector<double> vehicle_telemetry : sensor_fusion) {
    size_t id = vehicle_telemetry[0];

    double s_p = Map::WrapDistance(vehicle_telemetry[5]);
    double d_p = vehicle_telemetry[6];

    if (d_p < 0) {  // Vehicle not on the rendered yet
      continue;
    }

    if (std::fabs(s_p - this->ego.state.s.p) > RANGE) {  // Vehicle out of sensor reach
      continue;
    }

    double v_x = vehicle_telemetry[3];
    double v_y = vehicle_telemetry[4];

    auto frenet_v = this->map.FrenetVelocity(s_p, d_p, v_x, v_y);

    Frenet state{{s_p, frenet_v.first, 0.0}, {d_p, frenet_v.second, 0.0}};

    size_t lane = Map::LaneIndex(state.d.p);
    lanes_traffic[lane].emplace_back(id, state);
  }

  this->lanes_traffic = lanes_traffic;

  for (size_t i = 0; i < this->lanes_traffic.size(); ++i) {
    auto &lane_traffic = this->lanes_traffic[i];
    // Sort the vehicles by s
    // TODO Since the map is circular it mught happen that vehicles past the 0 point are in front but are
    // here detected as behind..
    std::sort(lane_traffic.begin(), lane_traffic.end(), Vehicle::SComparator);
    std::cout << "Lane " << i << " traffic: " << this->lanes_traffic[i].size() << std::endl;
    std::cout << "Sorted Vehicles: ";
    for (auto &vehicle : this->lanes_traffic[i]) {
      std::cout << vehicle.state.s.p << " ";
    }
    std::cout << std::endl;
  }
}

void PathPlanning::PathPlanner::UpdatePredictions() {
  for (auto &lane_traffic : this->lanes_traffic) {
    for (auto &vehicle : lane_traffic) {
      vehicle.PredictTrajectory(TRAJECTORY_STEPS, TRAJECTORY_STEP_DT);
    }
  }
}

void PathPlanning::PathPlanner::UpdatePlan() {
  // TODO
}

PathPlanning::Frenet PathPlanning::PathPlanner::GetTarget(size_t lane, double t) const {
  const State &start_s = this->ego.state.s;
  const State &start_d = this->ego.state.d;

  // Limits the acceleration when going slower
  const double max_acc = start_s.v < MIN_SPEED ? MAX_ACC / 2.0 : MAX_ACC;
  // Velocity: v1 + a * t
  double s_v = std::min(MAX_SPEED, start_s.v + max_acc * t);
  // Projected Acceleration: (v2 - v1) / t
  double acc = (s_v - start_s.v) / t;
  // Constant acceleration: s1 + (v1 * t + 0.5 * a * t^2)
  double s_p = Map::WrapDistance(start_s.p + start_s.v * t + 0.5 * acc * std::pow(t, 2));
  // Final accelleration can be the projected one if going less than the speed limit
  double s_a = s_v < MAX_SPEED ? acc : 0.0;

  // If we have a vehicle ahead adapt the speed to avoid collisions
  Vehicle ahead;
  if (this->VehicleAhead(lane, ahead)) {
    const double distance = ahead.state.s.p - this->ego.state.s.p;
    std::cout << "[WARNING]: Vehicle " << ahead.id << " ahead at " << distance << " m" << std::endl;
    // Computes the distance at time t
    std::cout << ahead.trajectory.size() << std::endl;
    const double distance_at_t = ahead.trajectory.back().s.p - this->ego.StateAt(t).s.p;
    if (distance_at_t < SAFE_DISTANCE) {
      std::cout << "[WARNING]: Following " << ahead.id << std::endl;
      s_v = std::min(MAX_SPEED, ahead.state.s.v);  // Follow the car ahead
      s_a = 0.0;
      s_p = ahead.trajectory.back().s.p - SAFE_DISTANCE;
    }
  }

  const double d_p = Map::LaneDisplacement(lane);
  const double d_v = 0.0;
  const double d_a = 0.0;

  return {{s_p, s_v, s_a}, {d_p, d_v, d_a}};
}

bool PathPlanning::PathPlanner::VehicleAhead(size_t lane, Vehicle &vehicle) const {
  auto &lane_traffic = this->lanes_traffic[lane];
  std::cout << std::endl;
  auto it = std::lower_bound(lane_traffic.begin(), lane_traffic.end(), this->ego, Vehicle::SComparator);
  if (it != lane_traffic.end()) {
    vehicle = *it;
    return true;
  }
  return false;
}

void PathPlanning::PathPlanner::UpdateTrajectory() {
  // TODO behavior planning
  PathPlanning::Frenet start = this->ego.state;

  PathPlanning::Frenet target = GetTarget(ego.lane, TRAJECTORT_T);

  std::cout << "Start:" << std::endl;
  std::cout << "S: " << start.s.p << " - " << start.s.v << " - " << start.s.a << std::endl;
  std::cout << "D: " << start.d.p << " - " << start.d.v << " - " << start.d.a << std::endl;

  std::cout << "Target:" << std::endl;
  std::cout << "S: " << target.s.p << " - " << target.s.v << " - " << target.s.a << std::endl;
  std::cout << "D: " << target.d.p << " - " << target.d.v << " - " << target.d.a << std::endl;

  this->f_trajectory = this->trajectory_generator.Generate(start, target, TRAJECTORY_STEPS);

  std::cout << std::endl;
}

PathPlanning::Trajectory PathPlanning::PathPlanner::getGlobalCoordTrajectory() const {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  // std::cout << "Trajectory: " << std::endl;
  for (auto &step : this->f_trajectory) {
    auto coord = this->map.FrenetToCartesian(step.s.p, step.d.p);
    // std::cout << "[" << step.s.p << ", " << step.d.p << ", " << coord.first << ", " << coord.second << "] " << std::endl;
    // if (std::fabs(step.s.a) >= MAX_ACC || std::fabs(step.d.a) >= MAX_ACC) {
    //   std::cout << "[WARNING]: Acceleration exceeded (" << step.s.a << ", " << step.d.a << ")" << std::endl;
    // }
    // if (step.s.v >= MAX_SPEED || step.d.v >= MAX_SPEED) {
    //   std::cout << "[WARNING]: Speed exceeded (" << step.s.v << ", " << step.d.v << ")" << std::endl;
    // }
    next_x_vals.emplace_back(coord.first);
    next_y_vals.emplace_back(coord.second);
  }
  std::cout << std::endl;

  return Trajectory({next_x_vals, next_y_vals});
}
