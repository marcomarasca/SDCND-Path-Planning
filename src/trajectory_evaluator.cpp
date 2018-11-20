#include "trajectory_evaluator.h"

#include <iomanip>
#include <numeric>
#include "logger.h"
#include "map.h"
#include "vehicle.h"

PathPlanning::TrajectoryEvaluator::TrajectoryEvaluator(double max_speed, double step_dt)
    : max_speed(max_speed), step_dt(step_dt) {}

double PathPlanning::TrajectoryEvaluator::Evaluate(const FTrajectory &trajectory, const Traffic &traffic,
                                                   const Frenet &current_plan) const {
  if (trajectory.empty()) {
    return std::numeric_limits<double>::max();
  }

  size_t start_lane = Map::LaneIndex(trajectory.front().d.p);
  size_t target_lane = Map::LaneIndex(trajectory.back().d.p);

  if (Map::InvalidLane(start_lane) || Map::InvalidLane(target_lane)) {
    LOG(WARN) << LOG_BUFER << "Invalid Lane: " << trajectory.front().d.p << ", " << trajectory.back().d.p;
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
      lane_speed_cost = Logistic((max_speed - lane_speed) / max_speed);
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
  double t = trajectory.size() * this->step_dt;
  // Average speed cost, rewards higher average speed
  double speed = Map::ModDistance(trajectory.back().s.p, trajectory.front().s.p) / t;
  double speed_cost = Logistic((max_speed - speed) / max_speed);

  // Change plan cost
  size_t plan_lane = Map::LaneIndex(current_plan.d.p);
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