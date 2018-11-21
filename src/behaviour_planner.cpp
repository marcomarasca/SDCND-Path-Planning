#include "behaviour_planner.h"
#include <iomanip>
#include <iostream>
#include "logger.h"
#include "map.h"

PathPlanning::BehaviourPlanner::BehaviourPlanner(const TrajectoryGenerator &trajectory_generator, double max_speed,
                                                 double min_speed, double max_acc)
    : trajectory_generator(trajectory_generator),
      max_speed(max_speed),
      min_speed(min_speed),
      max_acc(max_acc),
      plan_evaluator(max_speed) {}

void PathPlanning::BehaviourPlanner::ResetPlan(const Frenet &state) {
  this->plan.target = state;
  this->plan.trajectory.clear();
  this->plan.t = 0.0;
};

PathPlanning::Plan PathPlanning::BehaviourPlanner::Update(const Vehicle &ego, const Traffic &traffic, double t,
                                                          double processing_time) {
  Plan best_plan;
  double min_cost = std::numeric_limits<double>::max();
  const size_t trajectory_length = this->trajectory_generator.TrajectoryLength(t);

  for (size_t target_lane : this->GetAvailableLanes(ego)) {
    LOG(DEBUG) << "Evaluating Lane: " << target_lane;

    // Generate a candidate plan
    Plan plan = this->GeneratePlan(ego, traffic, t, processing_time, target_lane);
    const double plan_cost = this->EvaluatePlan(plan, traffic);

    LOG(DEBUG) << "Cost for Lane " << target_lane << ": " << plan_cost;

    if (plan_cost < min_cost) {
      min_cost = plan_cost;
      best_plan = plan;
    }
  }

  if (!best_plan.trajectory.empty()) {
    this->plan = best_plan;
    LOG(INFO) << "<------ Best Lane: " << Map::LaneIndex(best_plan.target.d.p) << " (Cost: " << min_cost << ") ------>";
  } else {
    LOG(WARN) << "<------ Cannot Compute Candidate ------>";
  }

  return best_plan;
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

  const int lane_delta = static_cast<int>(target_lane) - static_cast<int>(start_lane);
  // Limits max speed when changing lanes + slightly reduce speed for outer lane (e.g. right lane is slower traffic)
  const double max_speed = this->max_speed - std::abs(lane_delta) - 0.2 * target_lane;
  // Limits the acceleration when going slower
  const double max_acc = start.s.v < this->min_speed ? this->max_acc / 2.0 : this->max_acc;

  LOG(DEBUG) << LOG_BUFFER << "Lane Constraints: " << max_speed << " m/s, " << max_acc << " m/s^2";

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
    const double safe_distance = this->SafeDistance(ahead.trajectory.back().s.v);

    LOG(DEBUG) << LOG_BUFFER << "Vehicle " << ahead.id << " Ahead in " << distance
               << " m (Safe Distance: " << safe_distance << " m)";

    // Max s delta at time t according to front vehicle position at t
    const double max_s_p_delta = Map::ModDistance(ahead.trajectory.back().s.p - safe_distance, start.s.p);
    if (max_s_p_delta < s_p_delta) {
      s_v = std::min(s_v, ahead.state.s.v);
      s_p_delta = max_s_p_delta;
      s_a = 0.0;

      if (s_p_delta < 0) {
        LOG(WARN) << "Collision Risk";
        s_p_delta = safe_distance;
      }

      LOG(DEBUG) << LOG_BUFFER << "Following Vehicle " << ahead.id << " at Speed: " << s_v << " (Delta: " << s_p_delta
                 << ", Max Delta: " << max_s_p_delta << ")";
    }
  }

  const double s_p = start.s.p + s_p_delta;
  double d_p = Map::LaneDisplacement(target_lane);
  // Limits the amount of displacement to reduce jerking
  if (std::fabs(d_p - start.d.p) >= LANE_WIDTH) {
    d_p = start.d.p + lane_delta * LANE_WIDTH;
  }
  const double d_v = 0.0;
  const double d_a = 0.0;

  return {{s_p, s_v, s_a}, {d_p, d_v, d_a}};
}

PathPlanning::Plan PathPlanning::BehaviourPlanner::GeneratePlan(const Vehicle &ego, const Traffic &traffic, double t,
                                                                double processing_time, size_t target_lane) const {
  // Generates a trajectory considering the delay given by the processing time
  size_t forward_steps = std::min(ego.trajectory.size(), this->trajectory_generator.TrajectoryLength(processing_time));

  LOG(DEBUG) << LOG_BUFFER << "Forward Steps: " << forward_steps;

  // Generates a target for the given lane
  Frenet target = this->PredictTarget(ego, traffic, target_lane, t);
  size_t trajectory_length = this->trajectory_generator.TrajectoryLength(t);

  if (forward_steps == 0) {
    return {target, this->trajectory_generator.Generate(ego.state, target, trajectory_length), t};
  }

  Frenet start = ego.StateAt(forward_steps - 1);

  // Fix for map wrapping
  if (start.s.p >= target.s.p) {
    LOG(WARN) << "Target Before Starting State: " << target.s.p << " - " << start.s.p;
    target.s.p += MAP_MAX_S;
  }

  // Copies the head of the trajectory till
  FTrajectory trajectory{ego.trajectory.begin(), ego.trajectory.begin() + forward_steps};
  trajectory.reserve(trajectory_length);

  // Generates the tail of the trajectory
  FTrajectory trajectory_tail = this->trajectory_generator.Generate(start, target, trajectory_length - forward_steps);

  // Append the tail to the original trajectory
  trajectory.insert(trajectory.end(), trajectory_tail.begin(), trajectory_tail.end());

  return {target, trajectory, t};
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

double PathPlanning::BehaviourPlanner::SafeDistance(double v) const {
  return std::pow(v, 2) / (2 * this->max_acc) + 2 * VEHICLE_LENGTH;
}

double PathPlanning::BehaviourPlanner::EvaluatePlan(const Plan &plan, const Traffic &traffic) const {
  return this->plan_evaluator.Evaluate(plan, this->plan, traffic);
}