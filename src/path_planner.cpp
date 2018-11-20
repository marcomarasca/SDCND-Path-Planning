#include "path_planner.h"
#include <algorithm>
#include <memory>
#include <set>
#include "utils.h"

double PathPlanning::PathPlanner::MaxSpeed(size_t lane_index) { 
  // TODO check this workaround
  return MAX_SPEED - 1 * lane_index; 
}

PathPlanning::PathPlanner::PathPlanner(Map &map, size_t lane_n)
    : map(map), ego(EGO_ID), traffic(lane_n), trajectory_generator(TRAJECTORY_STEP_DT) {}

void PathPlanning::PathPlanner::Update(const json &telemetry) {
  this->UpdateEgo(telemetry);
  this->UpdateTraffic(telemetry);
  this->UpdatePredictions();
  this->UpdatePlan();
  this->UpdateTrajectory();
}

void PathPlanning::PathPlanner::UpdateEgo(const json &telemetry) {
  const int steps_to_go = telemetry["previous_path_x"].size();
  const int steps_consumed = this->ego.trajectory.size() - steps_to_go;

  const double s_p = telemetry["s"];
  const double s_v = Mph2ms(telemetry["speed"]);
  const double d_p = telemetry["d"];

  Frenet state{{s_p, s_v, 0.0}, {d_p, 0.0, 0.0}};

  std::cout << "Prev Path: " << steps_to_go << std::endl;
  std::cout << "Steps Consumed: " << steps_consumed << std::endl;
  std::cout << "Telemetry: (S: " << state.s.p << ", D: " << state.d.p << ", Speed: " << state.s.v << ")" << std::endl;

  if (steps_to_go == 0) {
    // Reset plan (keeps current state as target)
    this->plan = state;
    this->ego.trajectory.clear();
  } else if (steps_consumed > 0) {
    // Use data from the previous trajectory
    state = this->ego.StateAt(steps_consumed - 1);
  }

  this->ego.UpdateState(state);
  std::cout << "State: (S: " << this->ego.state.s.p << ", D: " << this->ego.state.d.p
            << ", S_V: " << this->ego.state.s.v << ", D_V: " << this->ego.state.d.v << ")" << std::endl;
}

void PathPlanning::PathPlanner::UpdateTraffic(const json &telemetry) {
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  // Reserve some space in the lane
  for (auto &lane : this->traffic) {
    lane.clear();
    lane.reserve(sensor_fusion.size() / traffic.size());
  }

  // Reads sensor fusion data and maps it to the current vehicles
  for (std::vector<double> vehicle_telemetry : sensor_fusion) {
    // Vehicle telemetry: id, x, y, vx, vy, s, d,
    int id = vehicle_telemetry[0];

    double s_p = vehicle_telemetry[5];
    double d_p = vehicle_telemetry[6];

    if (d_p < 0) {  // Vehicle not on the rendered yet
      continue;
    }

    size_t lane = Map::LaneIndex(d_p);

    if (Map::InvalidLane(lane)) {
      continue;
    }

    if (std::fabs(Map::ModDistance(s_p, this->ego.state.s.p)) > RANGE) {  // Vehicle out of sensor reach
      continue;
    }

    double v_x = vehicle_telemetry[3];
    double v_y = vehicle_telemetry[4];

    auto frenet_v = this->map.FrenetVelocity(s_p, d_p, v_x, v_y);

    Frenet state{{s_p, frenet_v.first, 0.0}, {d_p, frenet_v.second, 0.0}};

    this->traffic[lane].emplace_back(id, state);
  }

  const double s = this->ego.state.s.p;
  for (auto &lane_traffic : this->traffic) {
    // Sort the vehicles by distance to the ego vehicle
    std::sort(lane_traffic.begin(), lane_traffic.end(), [&s](const Vehicle &a, const Vehicle &b) {
      return Map::ModDistance(a.state.s.p, s) < Map::ModDistance(b.state.s.p, s);
    });
    // std::cout << "Lane Traffic: ";
    // for (auto &vehicle : lane_traffic) {
    //   std::cout << vehicle.id << ":" << vehicle.state.s.p << " ";
    // }
    // std::cout << std::endl;
  }
}

void PathPlanning::PathPlanner::UpdatePredictions() {
  for (auto &lane_traffic : this->traffic) {
    for (auto &vehicle : lane_traffic) {
      vehicle.PredictTrajectory(TRAJECTORY_STEPS, TRAJECTORY_STEP_DT);
    }
  }
}

void PathPlanning::PathPlanner::UpdatePlan() {
  // Get target lanes
  const size_t current_lane = this->ego.GetLane();
  std::vector<size_t> available_lanes;
  available_lanes.reserve(LANES_N);
  available_lanes.emplace_back(current_lane);
  if (current_lane > 0) {
    available_lanes.emplace_back(current_lane - 1);
  }
  if (current_lane < LANES_N - 1) {
    available_lanes.emplace_back(current_lane + 1);
  }

  double min_cost = std::numeric_limits<double>::max();
  Frenet best_target;
  FTrajectory best_trajectory;
  std::cout << "Computing best lane.." << std::endl;
  for (size_t target_lane : available_lanes) {
    std::cout << "Computing cost for lane: " << target_lane << std::endl;
    // Generate a candidate trajectory
    Frenet target = this->PredictTarget(target_lane, TRAJECTORY_T);
    FTrajectory trajectory = this->trajectory_generator.Generate(this->ego.state, target, TRAJECTORY_STEPS);
    const double trajectory_cost = this->TrajectoryCost(trajectory);
    std::cout << "Cost for lane " << target_lane << ": " << trajectory_cost << std::endl;
    if (trajectory_cost < min_cost) {
      min_cost = trajectory_cost;
      best_trajectory = trajectory;
      best_target = target;
    }
  }
  if (!best_trajectory.empty()) {
    std::cout << "Best target lane: " << Map::LaneIndex(best_target.d.p) << std::endl;
    std::cout << "Best trajectory: (Length " << best_trajectory.size() << ", Cost: " << min_cost << ")" << std::endl;
    this->plan = best_target;
  } else {
    std::cout << "Could not find an optimal trajectory" << std::endl;
  }
}

double PathPlanning::PathPlanner::TrajectoryCost(const FTrajectory &trajectory) const {
  if (trajectory.empty()) {
    return std::numeric_limits<double>::max();
  }

  size_t start_lane = Map::LaneIndex(trajectory.front().d.p);
  size_t target_lane = Map::LaneIndex(trajectory.back().d.p);

  if (Map::InvalidLane(start_lane) || Map::InvalidLane(target_lane)) {
    std::cout << "[WARNING]: Invalid lane (" << start_lane << ", " << target_lane << ")" << std::endl;
    return std::numeric_limits<double>::max();
  }

  // Collision cost
  double min_distance = std::numeric_limits<double>::max();
  bool collision = false;
  Vehicle closest_vehicle(EGO_ID);
  for (const auto &lane_traffic : this->traffic) {
    for (const auto &vehicle : lane_traffic) {
      double closest_distance = std::numeric_limits<double>::max();
      // Check the vehicle trajectory
      auto &other_trajectory = vehicle.trajectory;
      for (size_t i = 0; i < trajectory.size(); ++i) {
        double s_distance = std::fabs(Map::ModDistance(trajectory[i].s.p, other_trajectory[i].s.p));
        double d_distance = std::fabs(trajectory[i].d.p - other_trajectory[i].d.p);
        double distance_at_t = Distance(0, 0, d_distance, s_distance);
        if (distance_at_t < closest_distance) {
          closest_distance = distance_at_t;
        }
        if (s_distance < VEHICLE_LENGTH * 2 &&
            Map::LaneIndex(trajectory[i].d.p) == Map::LaneIndex(other_trajectory[i].d.p)) {
          std::cout << "[COLLISION]: (" << trajectory[i].s.p << ", " << trajectory[i].d.p << ") - ("
                    << other_trajectory[i].s.p << ", " << other_trajectory[i].d.p << ")" << std::endl;
          collision = true;
          break;
        }
      }
      if (closest_distance < min_distance) {
        min_distance = closest_distance;
        closest_vehicle = vehicle;
      }
      if (collision) {
        break;
      }
    }
    if (collision) {
      break;
    }
  }

  double collision_cost = 0.0;
  double buffer_cost = 0.0;

  if (collision) {
    collision_cost = 1.0;
    std::cout << "[WARNING]: Collision on trajectory with vehicle " << closest_vehicle.id << " ("
              << closest_vehicle.state.s.p << ", " << closest_vehicle.state.s.v << ")" << std::endl;
  } else {
    collision_cost = 0.0;
  }

  buffer_cost = Logistic(VEHICLE_LENGTH * 2 / min_distance);

  std::cout << "Collistion cost: " << collision_cost << " (Min distance: " << min_distance << ")" << std::endl;
  std::cout << "Danger cost: " << buffer_cost << " (Min distance: " << min_distance << ")" << std::endl;

  // Traffic speed cost
  double lane_speed_cost = 0.0;
  double lane_speed = std::numeric_limits<double>::max();
  if (!this->traffic[target_lane].empty()) {
    lane_speed = 0.0;
    size_t vehicles_n = 0;
    for (auto &vehicle : this->traffic[target_lane]) {
      if (Map::ModDistance(vehicle.state.s.p, this->ego.state.s.p) > 0) {
        lane_speed += vehicle.state.s.v;
        vehicles_n++;
      }
    }
    if (vehicles_n > 0) {
      lane_speed /= vehicles_n;
      lane_speed_cost = Logistic((MAX_SPEED - lane_speed) / MAX_SPEED);
    }
  }
  std::cout << "Lane speed cost: " << lane_speed_cost << " (Lane speed ahead: " << lane_speed << ")" << std::endl;

  // Lane traffic cost
  double lane_traffic_cost = 0.0;
  size_t tot_traffic =
      std::accumulate(this->traffic.begin(), this->traffic.end(), 0,
                      [](const size_t &value, const LaneTraffic &lane_traffic) { return value + lane_traffic.size(); });
  double lane_traffic = this->traffic[target_lane].size();
  if (tot_traffic > 0) {
    lane_traffic_cost = Logistic((double)this->traffic[target_lane].size() / tot_traffic);
  }
  std::cout << "Traffic cost: " << lane_traffic_cost << "(Lane traffic: " << lane_traffic
            << ", Tot Traffic: " << tot_traffic << ")" << std::endl;

  // Trajectory time
  double t = trajectory.size() * TRAJECTORY_STEP_DT;
  // Average speed cost, rewards higher average speed
  double speed = Map::ModDistance(trajectory.back().s.p, this->ego.state.s.p) / t;
  double speed_cost = Logistic((MAX_SPEED - speed) / MAX_SPEED);
  std::cout << "Speed cost: " << speed_cost << " (Avg speed: " << speed << ")" << std::endl;

  // Change plan cost
  size_t plan_lane = Map::LaneIndex(this->plan.d.p);
  double change_plan_cost = plan_lane != target_lane ? 1.0 : 0.0;
  std::cout << "Change plan cost: " << change_plan_cost << std::endl;

  double unfinished_plan_cost = plan_lane != start_lane ? 1.0 : 0.0;
  std::cout << "Unfinished plan cost: " << unfinished_plan_cost << std::endl;

  double cost = 0.0;
  cost += 10000 * collision_cost;
  cost += 5000 * unfinished_plan_cost;
  cost += 500 * speed_cost;
  cost += 250 * lane_traffic_cost;
  cost += 200 * lane_speed_cost;
  cost += 100 * buffer_cost;
  cost += 50 * change_plan_cost;

  return cost;
}

PathPlanning::Frenet PathPlanning::PathPlanner::PredictTarget(size_t target_lane, double t) const {
  const State &start_s = this->ego.state.s;
  const State &start_d = this->ego.state.d;

  size_t start_lane = this->ego.GetLane();

  int lane_diff = static_cast<int>(target_lane) - static_cast<int>(start_lane);
  const double max_speed = MaxSpeed(target_lane);
  // Limits the acceleration when going slower
  const double max_acc = start_s.v < MIN_SPEED ? MAX_ACC / 2.0 : MAX_ACC - 0.2 * std::abs(lane_diff);
  std::cout << "Lane " << target_lane << " Max speed: " << max_speed << ", Max Acc: " << max_acc << std::endl;
  // Velocity: v1 + a * t
  double s_v = std::min(max_speed, start_s.v + max_acc * t);
  // Projected Acceleration: (v2 - v1) / t
  double acc = (s_v - start_s.v) / t;
  // Constant acceleration: s1 + (v1 * t + 0.5 * a * t^2)
  double s_p_delta = start_s.v * t + 0.5 * acc * std::pow(t, 2);
  // Final accelleration can be the projected one if going less than the speed limit
  double s_a = 0.0;  // s_v < max_speed ? (max_speed - s_v) / TRAJECTORY_STEP_DT : 0.0;

  // If we have a vehicle ahead adapt the speed to avoid collisions
  Vehicle ahead(EGO_ID);
  if (this->VehicleAhead(target_lane, ahead)) {
    const double distance = Map::ModDistance(ahead.state.s.p, start_s.p);
    std::cout << "[WARNING]: Vehicle " << ahead.id << " ahead at " << distance << " m" << std::endl;
    // Max s delta at time t according to front vehicle position at t
    const double max_s_p_delta = Map::ModDistance(ahead.trajectory.back().s.p - SAFE_DISTANCE, start_s.p);
    if (max_s_p_delta < s_p_delta) {
      std::cout << "[WARNING]: Following " << ahead.id << "(Delta: " << s_p_delta << ", Max: " << max_s_p_delta << ")"
                << std::endl;
      s_p_delta = max_s_p_delta;
      s_v = std::min(s_v, ahead.state.s.v);  // Follow the car ahead
      s_a = 0.0;
    }
  }

  const double s_p = start_s.p + s_p_delta;
  double d_p = Map::LaneDisplacement(target_lane);
  if (std::fabs(d_p - start_d.p) >= LANE_WIDTH) {
    d_p = start_d.p + lane_diff * LANE_WIDTH;
  }
  const double d_v = 0.0;
  const double d_a = 0.0;

  return {{s_p, s_v, s_a}, {d_p, d_v, d_a}};
}

bool PathPlanning::PathPlanner::VehicleAhead(size_t lane, Vehicle &ahead) const {
  if (Map::InvalidLane(lane)) {
    return false;
  }
  auto &lane_traffic = this->traffic[lane];
  bool found = false;
  // Lane vehicles are sorted by distance to the ego vehicle
  for (auto &vehicle : lane_traffic) {
    double distance = Map::ModDistance(vehicle.state.s.p, this->ego.state.s.p);
    if (distance >= 0) {
      ahead = vehicle;
      found = true;
      break;
    }
  }
  return found;
}

void PathPlanning::PathPlanner::UpdateTrajectory() {
  // TODO behavior planning
  PathPlanning::Frenet start = this->ego.state;

  PathPlanning::Frenet &target = this->plan;  // GetTarget(ego.GetLane(), TRAJECTORY_T);

  std::cout << "Start:" << std::endl;
  std::cout << "S: " << start.s.p << " - " << start.s.v << " - " << start.s.a << std::endl;
  std::cout << "D: " << start.d.p << " - " << start.d.v << " - " << start.d.a << std::endl;

  std::cout << "Target:" << std::endl;
  std::cout << "S: " << target.s.p << " - " << target.s.v << " - " << target.s.a << std::endl;
  std::cout << "D: " << target.d.p << " - " << target.d.v << " - " << target.d.a << std::endl;

  this->ego.UpdateTrajectory(this->trajectory_generator.Generate(start, target, TRAJECTORY_STEPS));

  std::cout << std::endl;
}

PathPlanning::Trajectory PathPlanning::PathPlanner::getGlobalCoordTrajectory() const {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  // std::cout << "Trajectory: " << std::endl;
  for (auto &step : this->ego.trajectory) {
    auto coord = this->map.FrenetToCartesian(step.s.p, step.d.p);
    // std::cout << "[" << step.s.p << ", " << step.d.p << ", " << coord.first << ", " << coord.second << "] " <<
    // std::endl; if (std::fabs(step.s.a) >= MAX_ACC || std::fabs(step.d.a) >= MAX_ACC) {
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
