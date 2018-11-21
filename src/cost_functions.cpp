#include "cost_functions.h"

#include <iomanip>
#include <numeric>
#include "logger.h"
#include "map.h"
#include "vehicle.h"

double PathPlanning::CostFunctions::CollisionCost(const FTrajectory &trajectory, const Frenet &current_plan,
                                                  const Collision &collision, const TrafficData &traffic_data,
                                                  double max_speed, double step_dt) {
  double collision_cost = 0.0;

  if (collision.first) {
    collision_cost = 1.0;
  }

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Collistion Cost: " << std::setw(COST_LOG_W)
             << collision_cost << " (Min Distance: " << collision.second << ")";

  return collision_cost;
}

double PathPlanning::CostFunctions::BufferCost(const FTrajectory &trajectory, const Frenet &current_plan,
                                               const Collision &collision, const TrafficData &traffic_data,
                                               double max_speed, double step_dt) {
  double buffer_cost = Logistic(VEHICLE_LENGTH * 2 / collision.second);

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Buffer Cost: " << std::setw(COST_LOG_W)
             << buffer_cost << " (Min Distance: " << collision.second << ")";

  return buffer_cost;
}

double PathPlanning::CostFunctions::AverageSpeedCost(const FTrajectory &trajectory, const Frenet &current_plan,
                                                     const Collision &collision, const TrafficData &traffic_data,
                                                     double max_speed, double step_dt) {
  // Trajectory time
  double t = trajectory.size() * step_dt;
  // Average speed cost, rewards higher average speed
  double speed = Map::ModDistance(trajectory.back().s.p, trajectory.front().s.p) / t;
  double speed_cost = Logistic((max_speed - speed) / max_speed);

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Speed Cost: " << std::setw(COST_LOG_W)
             << speed_cost << " (Avg speed: " << speed << ")";

  return speed_cost;
}

double PathPlanning::CostFunctions::LaneSpeedCost(const FTrajectory &trajectory, const Frenet &current_plan,
                                                  const Collision &collision, const TrafficData &traffic_data,
                                                  double max_speed, double step_dt) {
  double lane_speed_cost = 0.0;

  if (traffic_data.lane_traffic > 0) {
    lane_speed_cost = Logistic((max_speed - traffic_data.lane_speed) / max_speed);
  }

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Lane Speed Cost: " << std::setw(COST_LOG_W)
             << lane_speed_cost << " (Lane Speed: " << traffic_data.lane_speed << ")";

  return lane_speed_cost;
}

double PathPlanning::CostFunctions::LaneTrafficCost(const FTrajectory &trajectory, const Frenet &current_plan,
                                                    const Collision &collision, const TrafficData &traffic_data,
                                                    double max_speed, double step_dt) {
  // Lane traffic cost
  double lane_traffic_cost = 0.0;

  if (traffic_data.tot_traffic > 0) {
    lane_traffic_cost = Logistic(static_cast<double>(traffic_data.lane_traffic) / traffic_data.tot_traffic);
  }

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Traffic Cost: " << std::setw(COST_LOG_W)
             << lane_traffic_cost << " (Lane Traffic: " << traffic_data.lane_traffic
             << ", Total Traffic: " << traffic_data.tot_traffic << ")";

  return lane_traffic_cost;
}

double PathPlanning::CostFunctions::ChangePlanCost(const FTrajectory &trajectory, const Frenet &current_plan,
                                                   const Collision &collision, const TrafficData &traffic_data,
                                                   double max_speed, double step_dt) {
  // Change plan cost
  size_t plan_lane = Map::LaneIndex(current_plan.d.p);
  size_t target_lane = Map::LaneIndex(trajectory.back().d.p);

  double change_plan_cost = plan_lane != target_lane ? 1.0 : 0.0;

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Change Plan Cost: " << std::setw(COST_LOG_W)
             << change_plan_cost;

  return change_plan_cost;
}

double PathPlanning::CostFunctions::UnfinishedPlanCost(const FTrajectory &trajectory, const Frenet &current_plan,
                                                       const Collision &collision, const TrafficData &traffic_data,
                                                       double max_speed, double step_dt) {
  size_t plan_lane = Map::LaneIndex(current_plan.d.p);
  size_t start_lane = Map::LaneIndex(trajectory.front().d.p);
  double unfinished_plan_cost = plan_lane != start_lane ? 1.0 : 0.0;

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER)
             << "Unfinished Plan Cost: " << std::setw(COST_LOG_W) << unfinished_plan_cost;

  return unfinished_plan_cost;
}