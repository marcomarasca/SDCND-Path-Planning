#include "behaviour_planner.h"
#include <iomanip>
#include <iostream>
#include <numeric>
#include "logger.h"
#include "map.h"

double PathPlanning::BehaviourPlanner::SafeDistance(double v) {
  return std::pow(v, 2) / (2 * MAX_ACC) + 2 * VEHICLE_LENGTH;
}

PathPlanning::BehaviourPlanner::BehaviourPlanner(const TrajectoryGenerator &trajectory_generator)
    : trajectory_generator(trajectory_generator) {}

void PathPlanning::BehaviourPlanner::ResetPlan(const Frenet &state) { this->plan = state; };

PathPlanning::FTrajectory PathPlanning::BehaviourPlanner::UpdatePlan(const Vehicle &ego, const Traffic &traffic,
                                                                     size_t trajectory_steps, double processing_time) {
  const double t = trajectory_steps * this->trajectory_generator.step_dt;

  Frenet best_target;
  FTrajectory best_trajectory;
  double min_cost = std::numeric_limits<double>::max();

  for (size_t target_lane : this->GetAvailableLanes(ego)) {
    LOG(DEBUG) << "Computing Cost for Lane: " << target_lane;

    // Generate a candidate trajectory
    Frenet target = this->PredictTarget(ego, traffic, target_lane, t);
    FTrajectory trajectory = this->GenerateTrajectory(ego, target, trajectory_steps, processing_time);
    const double trajectory_cost = this->EvaluateTrajectory(trajectory, traffic);

    LOG(DEBUG) << "Cost for Lane " << target_lane << ": " << trajectory_cost;

    if (trajectory_cost < min_cost) {
      min_cost = trajectory_cost;
      best_trajectory = trajectory;
      best_target = target;
    }
  }

  if (!best_trajectory.empty()) {
    this->plan = best_target;

    LOG(INFO) << "<------ Best Lane: " << Map::LaneIndex(best_target.d.p) << " (Cost: " << min_cost << ") ------>";
  } else {
    LOG(WARN) << "<------ Cannot Compute Candidate ------>";
  }

  return best_trajectory;
}

std::vector<size_t> PathPlanning::BehaviourPlanner::GetAvailableLanes(const Vehicle &vehicle) const {
  // Get target lanes
  const size_t current_lane = vehicle.GetLane();
  std::vector<size_t> available_lanes;
  available_lanes.reserve(LANES_N);
  available_lanes.emplace_back(current_lane);
  if (current_lane > 0) {
    available_lanes.emplace_back(current_lane - 1);
  }
  if (current_lane < LANES_N - 1) {
    available_lanes.emplace_back(current_lane + 1);
  }
  return available_lanes;
}

PathPlanning::Frenet PathPlanning::BehaviourPlanner::PredictTarget(const Vehicle &ego, const Traffic &traffic,
                                                                   size_t target_lane, double t) const {
  auto &start = ego.state;
  size_t start_lane = Map::LaneIndex(start.d.p);

  int lane_diff = std::abs(static_cast<int>(target_lane) - static_cast<int>(start_lane));
  const double max_speed = MAX_SPEED - 1 * lane_diff;
  // Limits the acceleration when going slower
  const double max_acc = start.s.v < MIN_SPEED ? MAX_ACC / 2.0 : MAX_ACC - 0.2 * lane_diff;

  LOG(DEBUG) << LOG_BUFER << "Lane Constraints: " << max_speed << " m/s, " << max_acc << " m/s^2";

  // Velocity: v1 + a * t
  double s_v = std::min(max_speed, start.s.v + max_acc * t);
  // Projected Acceleration: (v2 - v1) / t
  double acc = (s_v - start.s.v) / t;
  // Constant acceleration: s1 + (v1 * t + 0.5 * a * t^2)
  double s_p_delta = start.s.v * t + 0.5 * acc * std::pow(t, 2);
  // Final acceleration can be the projected one if going less than the speed limit
  double s_a = 0.0;  // s_v < max_speed ? (max_speed - s_v) / TRAJECTORY_STEP_DT : 0.0;

  // If we have a vehicle ahead adapt the speed to avoid collisions
  Vehicle ahead(-1);
  if (this->GetVehicleAhead(ego, traffic, target_lane, ahead)) {
    const double distance = Map::ModDistance(ahead.state.s.p, start.s.p);
    const double safe_distance = SafeDistance(ahead.trajectory.back().s.v);

    LOG(DEBUG) << LOG_BUFER << "Vehicle " << ahead.id << " Ahead in " << distance
               << " m (Safe Distance: " << safe_distance << " m)";

    // Max s delta at time t according to front vehicle position at t
    const double max_s_p_delta = Map::ModDistance(ahead.trajectory.back().s.p - safe_distance, start.s.p);
    if (max_s_p_delta < s_p_delta) {
      s_v = std::min(s_v, ahead.state.s.v);

      LOG(DEBUG) << LOG_BUFER << "Following Vehicle " << ahead.id << " at Speed: " << s_v << " (Delta: " << s_p_delta
                 << ", Max Delta: " << max_s_p_delta << ")";

      s_p_delta = max_s_p_delta < 0 ? s_p_delta : max_s_p_delta;
      s_a = 0.0;
    }
  }

  const double s_p = start.s.p + s_p_delta;
  double d_p = Map::LaneDisplacement(target_lane);
  // Limits the amount of displacement to reduce jerking
  if (std::fabs(d_p - start.d.p) >= LANE_WIDTH) {
    d_p = start.d.p + lane_diff * LANE_WIDTH;
  }
  const double d_v = 0.0;
  const double d_a = 0.0;

  return {{s_p, s_v, s_a}, {d_p, d_v, d_a}};
}

PathPlanning::FTrajectory PathPlanning::BehaviourPlanner::GenerateTrajectory(const Vehicle &ego, const Frenet &target,
                                                                             size_t trajectory_steps,
                                                                             double processing_time) {
  // Generates a trajectory considering the delay given by the processing time
  size_t forward_steps =
      std::min(ego.trajectory.size(), static_cast<size_t>(processing_time / this->trajectory_generator.step_dt));

  LOG(DEBUG) << LOG_BUFER << "Forward Steps: " << forward_steps;

  if (forward_steps == 0) {
    return this->trajectory_generator.Generate(ego.state, target, trajectory_steps);
  }

  Frenet start = ego.StateAt(forward_steps - 1);
  FTrajectory trajectory{ego.trajectory.begin(), ego.trajectory.begin() + forward_steps};
  FTrajectory trajectory_tail = this->trajectory_generator.Generate(start, target, trajectory_steps - forward_steps);
  trajectory.reserve(trajectory_steps);
  trajectory.insert(trajectory.end(), trajectory_tail.begin(), trajectory_tail.end());

  return trajectory;
}

bool PathPlanning::BehaviourPlanner::GetVehicleAhead(const Vehicle &ego, const Traffic &traffic, size_t target_lane,
                                                     Vehicle &ahead) const {
  bool found = false;
  for (auto &vehicle : traffic[target_lane]) {
    double distance = Map::ModDistance(vehicle.state.s.p, ego.state.s.p);
    if (distance >= 0) {
      ahead = vehicle;
      found = true;
      // Lane vehicles are sorted by distance from the ego vehicle
      break;
    }
  }
  return found;
}

double PathPlanning::BehaviourPlanner::EvaluateTrajectory(const FTrajectory &trajectory, const Traffic &traffic) const {
  if (trajectory.empty()) {
    return std::numeric_limits<double>::max();
  }

  size_t start_lane = Map::LaneIndex(trajectory.front().d.p);
  size_t target_lane = Map::LaneIndex(trajectory.back().d.p);

  if (Map::InvalidLane(start_lane) || Map::InvalidLane(target_lane)) {
    LOG(WARN) << LOG_BUFER << "Invalid Lane: " << start_lane << ", " << target_lane;
    return std::numeric_limits<double>::max();
  }

  // Collision cost
  double min_distance = std::numeric_limits<double>::max();
  bool collision = false;
  Vehicle closest_vehicle(-1);
  for (const auto &lane_traffic : traffic) {
    for (const auto &vehicle : lane_traffic) {
      double closest_distance = std::numeric_limits<double>::max();
      // Check the vehicle trajectory
      auto &other_trajectory = vehicle.trajectory;
      assert(trajectory.size() == other_trajectory.size());

      for (size_t i = 0; i < trajectory.size(); ++i) {
        const auto &step = trajectory[i];
        const auto &other_step = other_trajectory[i];
        const double s_distance = std::fabs(Map::ModDistance(step.s.p, other_step.s.p));
        const double d_distance = std::fabs(step.d.p - other_step.d.p);
        const double distance_at_t = Distance(0, 0, d_distance, s_distance);
        if (distance_at_t < closest_distance) {
          closest_distance = distance_at_t;
        }
        if (s_distance < VEHICLE_LENGTH * 2 && Map::LaneIndex(step.d.p) == Map::LaneIndex(other_step.d.p)) {
          LOG(DEBUG) << LOG_BUFER << "Collision Detected: " << step.s.p << ", " << step.d.p << " - " << other_step.s.p
                     << ", " << other_step.d.p;
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
    LOG(DEBUG) << LOG_BUFER << "Collision with Vehicle " << closest_vehicle.id << " (" << closest_vehicle.state.s.p
               << ", " << closest_vehicle.state.s.v << ")";
  } else {
    collision_cost = 0.0;
  }

  buffer_cost = Logistic(VEHICLE_LENGTH * 2 / min_distance);

  // Traffic speed cost
  double lane_speed_cost = 0.0;
  double lane_speed = std::numeric_limits<double>::max();
  size_t lane_traffic = 0;
  if (!traffic[target_lane].empty()) {
    lane_speed = 0.0;
    double start_s = trajectory.front().s.p;
    for (auto &vehicle : traffic[target_lane]) {
      if (Map::ModDistance(vehicle.state.s.p, start_s) > 0) {
        lane_speed += vehicle.state.s.v;
        lane_traffic++;
      }
    }
    if (lane_traffic > 0) {
      lane_speed /= lane_traffic;
      lane_speed_cost = Logistic((MAX_SPEED - lane_speed) / MAX_SPEED);
    }
  }

  // Lane traffic cost
  double lane_traffic_cost = 0.0;
  size_t tot_traffic =
      std::accumulate(traffic.begin(), traffic.end(), 0,
                      [](const size_t &value, const LaneTraffic &lane_traffic) { return value + lane_traffic.size(); });
  if (tot_traffic > 0) {
    lane_traffic_cost = Logistic(static_cast<double>(lane_traffic) / tot_traffic);
  }

  // Trajectory time
  double t = trajectory.size() * this->trajectory_generator.step_dt;
  // Average speed cost, rewards higher average speed
  double speed = Map::ModDistance(trajectory.back().s.p, trajectory.front().s.p) / t;
  double speed_cost = Logistic((MAX_SPEED - speed) / MAX_SPEED);

  // Change plan cost
  size_t plan_lane = Map::LaneIndex(this->plan.d.p);
  double change_plan_cost = plan_lane != target_lane ? 1.0 : 0.0;

  double unfinished_plan_cost = plan_lane != start_lane ? 1.0 : 0.0;

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Collistion Cost: " << std::setw(COST_LOG_W)
             << collision_cost << " (Min Distance: " << min_distance << ")";
  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Buffer Cost: " << std::setw(COST_LOG_W)
             << buffer_cost << " (Min Distance: " << min_distance << ")";
  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Lane Speed Cost: " << std::setw(COST_LOG_W)
             << lane_speed_cost << " (Lane Speed: " << lane_speed << ")";
  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Traffic Cost: " << std::setw(COST_LOG_W)
             << lane_traffic_cost << " (Lane Traffic: " << lane_traffic << ", Total Traffic: " << tot_traffic << ")";
  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Speed Cost: " << std::setw(COST_LOG_W)
             << speed_cost << " (Avg speed: " << speed << ")";
  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Change Plan Cost: " << std::setw(COST_LOG_W)
             << change_plan_cost;
  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER)
             << "Unfinished Plan Cost: " << std::setw(COST_LOG_W) << unfinished_plan_cost;

  double cost = 0.0;
  cost += 10000 * collision_cost;
  cost += 5000 * unfinished_plan_cost;
  cost += 1000 * speed_cost;
  cost += 250 * lane_traffic_cost;
  cost += 200 * lane_speed_cost;
  cost += 50 * buffer_cost;
  cost += 10 * change_plan_cost;

  return cost;
}