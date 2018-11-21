#include "trajectory_evaluator.h"

#include "logger.h"
#include "map.h"
#include "vehicle.h"

PathPlanning::TrajectoryEvaluator::TrajectoryEvaluator(double max_speed, double step_dt)
    : max_speed(max_speed),
      step_dt(step_dt),
      cost_functions({{CostFunctions::CollisionCost, COLLISION_COST_W},
                      {CostFunctions::UnfinishedPlanCost, UNFINISHED_PLAN_COST_W},
                      {CostFunctions::AverageSpeedCost, SPEED_COST_W},
                      {CostFunctions::LaneTrafficCost, LANE_TRAFFIC_COST_W},
                      {CostFunctions::LaneSpeedCost, LANE_SPEED_COST_W},
                      {CostFunctions::BufferCost, BUFFER_COST_W},
                      {CostFunctions::ChangePlanCost, CHANGE_PLAN_COST_W}}) {}

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

  Collision collision = this->DetectCollision(trajectory, traffic);
  TrafficData traffic_data = this->GetTrafficData(trajectory, traffic);

  double cost = 0.0;

  for (auto &cost_function : this->cost_functions) {
    cost += cost_function.second *
            cost_function.first(trajectory, current_plan, collision, traffic_data, this->max_speed, this->step_dt);
  }

  return cost;
}

PathPlanning::TrafficData PathPlanning::TrajectoryEvaluator::GetTrafficData(const FTrajectory &trajectory,
                                                                            const Traffic &traffic) const {
  size_t target_lane = Map::LaneIndex(trajectory.back().d.p);
  size_t lane_traffic = 0;
  double lane_speed = std::numeric_limits<double>::max();
  size_t tot_traffic = 0;

  for (size_t i = 0; i < traffic.size(); ++i) {
    auto &lane = traffic[i];
    tot_traffic += lane.size();
    if (i == target_lane && !lane.empty()) {
      lane_speed = 0.0;
      const double start_s = trajectory.front().s.p;
      for (auto &vehicle : traffic[target_lane]) {
        if (Map::ModDistance(vehicle.state.s.p, start_s) > 0) {
          lane_speed += vehicle.state.s.v;
          lane_traffic++;
        }
      }
      if (lane_traffic > 0) {
        lane_speed /= lane_traffic;
      } else {
        lane_speed = std::numeric_limits<double>::max();
      }
    }
  }

  return {target_lane, lane_traffic, lane_speed, tot_traffic};
}

PathPlanning::Collision PathPlanning::TrajectoryEvaluator::DetectCollision(const FTrajectory &trajectory1,
                                                                           const FTrajectory &trajectory2) const {
  assert(trajectory1.size() == trajectory2.size());

  double closest_distance = std::numeric_limits<double>::max();
  bool collision = false;
  for (size_t i = 0; i < trajectory1.size(); ++i) {
    const auto &step = trajectory1[i];
    const auto &other_step = trajectory2[i];

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
  return {collision, closest_distance};
}

PathPlanning::Collision PathPlanning::TrajectoryEvaluator::DetectCollision(const FTrajectory &trajectory,
                                                                           const Traffic &traffic) const {
  // Collision cost
  double min_distance = std::numeric_limits<double>::max();
  bool collision = false;
  Vehicle closest_vehicle(-1);
  for (const auto &lane_traffic : traffic) {
    for (const auto &vehicle : lane_traffic) {
      // Check the vehicle trajectory
      auto &other_trajectory = vehicle.trajectory;
      auto distance = this->DetectCollision(trajectory, other_trajectory);

      if (distance.second < min_distance) {
        min_distance = distance.second;
        closest_vehicle = vehicle;
      }

      if (distance.first) {
        collision = true;
        break;
      }
    }

    if (collision) {
      break;
    }
  }
  return {collision, min_distance};
}
