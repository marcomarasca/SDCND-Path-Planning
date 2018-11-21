#include "cost_functions.h"

#include <iomanip>
#include <numeric>
#include "logger.h"
#include "map.h"
#include "vehicle.h"

double PathPlanning::CostFunctions::CollisionCost(const FTrajectory &trajectory, double t, const Frenet &current_plan,
                                                  const Collision &collision, const TrafficData &traffic_data,
                                                  double max_speed) {
  double collision_cost = 0.0;

  if (collision.first) {
    collision_cost = 1.0;
  }

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Collistion Cost: " << std::setw(COST_LOG_W)
             << collision_cost << " (Min Distance: " << collision.second << ")";

  return collision_cost;
}

double PathPlanning::CostFunctions::BufferCost(const FTrajectory &trajectory, double t, const Frenet &current_plan,
                                               const Collision &collision, const TrafficData &traffic_data,
                                               double max_speed) {
  double buffer_cost = Logistic(VEHICLE_LENGTH * 2 / collision.second);

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Buffer Cost: " << std::setw(COST_LOG_W)
             << buffer_cost << " (Min Distance: " << collision.second << ")";

  return buffer_cost;
}

double PathPlanning::CostFunctions::AverageSpeedCost(const FTrajectory &trajectory, double t,
                                                     const Frenet &current_plan, const Collision &collision,
                                                     const TrafficData &traffic_data, double max_speed) {
  // Average speed cost, rewards higher average speed
  double speed = Map::ModDistance(trajectory.back().s.p, trajectory.front().s.p) / t;
  double speed_cost = Logistic((max_speed - speed) / max_speed);

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Speed Cost: " << std::setw(COST_LOG_W)
             << speed_cost << " (Avg speed: " << speed << ")";

  return speed_cost;
}

double PathPlanning::CostFunctions::LaneSpeedCost(const FTrajectory &trajectory, double t, const Frenet &current_plan,
                                                  const Collision &collision, const TrafficData &traffic_data,
                                                  double max_speed) {
  double lane_speed_cost = 0.0;

  if (traffic_data.lane_traffic > 0) {
    lane_speed_cost = Logistic((max_speed - traffic_data.lane_speed) / max_speed);
  }

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Lane Speed Cost: " << std::setw(COST_LOG_W)
             << lane_speed_cost << " (Lane Speed: " << traffic_data.lane_speed << ")";

  return lane_speed_cost;
}

double PathPlanning::CostFunctions::LaneTrafficCost(const FTrajectory &trajectory, double t, const Frenet &current_plan,
                                                    const Collision &collision, const TrafficData &traffic_data,
                                                    double max_speed) {
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

double PathPlanning::CostFunctions::ChangePlanCost(const FTrajectory &trajectory, double t, const Frenet &current_plan,
                                                   const Collision &collision, const TrafficData &traffic_data,
                                                   double max_speed) {
  // Change plan cost
  size_t plan_lane = Map::LaneIndex(current_plan.d.p);

  double change_plan_cost = plan_lane != traffic_data.target_lane ? 1.0 : 0.0;

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER) << "Change Plan Cost: " << std::setw(COST_LOG_W)
             << change_plan_cost;

  return change_plan_cost;
}

double PathPlanning::CostFunctions::UnfinishedPlanCost(const FTrajectory &trajectory, double t,
                                                       const Frenet &current_plan, const Collision &collision,
                                                       const TrafficData &traffic_data, double max_speed) {
  size_t plan_lane = Map::LaneIndex(current_plan.d.p);
  double d_distance = std::fabs(current_plan.d.p - trajectory.front().d.p);

  double unfinished_plan_cost = 0.0;

  if (traffic_data.target_lane != plan_lane && d_distance > LANE_WIDTH / 4) {
    double d_distance = std::fabs(current_plan.d.p - trajectory.front().d.p);
    unfinished_plan_cost = Logistic(d_distance / LANE_WIDTH);
  }

  LOG(DEBUG) << LOG_BUFER << std::left << std::setw(COST_LOG_BUFFER)
             << "Unfinished Plan Cost: " << std::setw(COST_LOG_W) << unfinished_plan_cost
             << " (Distance: " << d_distance << ")";

  return unfinished_plan_cost;
}