#include "path_planner.h"
#include <algorithm>
#include "logger.h"
#include "utils.h"

PathPlanning::PathPlanner::PathPlanner(Map &map, size_t lane_n)
    : map(map),
      ego(EGO_ID),
      traffic(lane_n),
      trajectory_generator(map, TRAJECTORY_STEP_DT),
      behaviour_planner(this->trajectory_generator) {}

void PathPlanning::PathPlanner::Update(const json &telemetry, double processing_time) {
  LOG(INFO) << "<------ Updating Path Planner (Processing time: " << processing_time << " s) ------>";
  this->UpdateEgo(telemetry);
  this->UpdateTraffic(telemetry);
  this->UpdatePredictions();
  this->UpdatePlan(processing_time);
}

void PathPlanning::PathPlanner::UpdateEgo(const json &telemetry) {
  const int steps_to_go = telemetry["previous_path_x"].size();
  const int steps_consumed = this->ego.trajectory.size() - steps_to_go;

  const double s_p = telemetry["s"];
  const double s_v = Mph2ms(telemetry["speed"]);
  const double d_p = telemetry["d"];

  Frenet state{{s_p, s_v, 0.0}, {d_p, 0.0, 0.0}};

  const size_t col_w = 12;

  LOG(INFO) << "Updating Vehicle State (Consumed Steps: " << steps_consumed << ", Unvisited Steps: " << steps_to_go
            << "): ";
  LOG(INFO) << LOG_BUFER << std::setw(col_w) << "Telemetry: " << std::setw(col_w) << "s_p" << std::setw(col_w) << "d_p"
            << std::setw(col_w) << "s_v" << std::setw(col_w) << "d_v";
  LOG(INFO) << LOG_BUFER << std::setw(col_w) << "Received: " << std::setw(col_w) << state.s.p << std::setw(col_w)
            << state.d.p << std::setw(col_w) << state.s.v << std::setw(col_w) << state.d.v;

  this->ego.UpdateState(state);

  if (steps_to_go == 0) {
    // Reset plan (keeps current state as target)
    this->behaviour_planner.ResetPlan(state);
    this->ego.ResetTrajectory();
  } else if (steps_consumed > 0) {
    // Use data from the previous trajectory
    this->ego.ForwardState(steps_consumed - 1);
  }

  LOG(INFO) << LOG_BUFER << std::setw(col_w) << "Using: " << std::setw(col_w) << this->ego.state.s.p << std::setw(col_w)
            << this->ego.state.d.p << std::setw(col_w) << this->ego.state.s.v << std::setw(col_w)
            << this->ego.state.d.v;
}

void PathPlanning::PathPlanner::UpdateTraffic(const json &telemetry) {
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  LOG(INFO) << "Updating Traffic Data (Sensed Vehicles: " << sensor_fusion.size() << ")";

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
  size_t lane = 0;
  for (auto &lane_traffic : this->traffic) {
    LOG(DEBUG) << LOG_BUFER << "Lane " << lane << " Traffic: " << lane_traffic.size();
    // Sort the vehicles by distance to the ego vehicle
    std::sort(lane_traffic.begin(), lane_traffic.end(), [&s](const Vehicle &a, const Vehicle &b) {
      return Map::ModDistance(a.state.s.p, s) < Map::ModDistance(b.state.s.p, s);
    });
    ++lane;
  }
}

void PathPlanning::PathPlanner::UpdatePredictions() {
  LOG(INFO) << "Updating Traffic Predictions";
  for (auto &lane_traffic : this->traffic) {
    for (auto &vehicle : lane_traffic) {
      FTrajectory trajectory = this->trajectory_generator.Predict(
          vehicle.state, [&vehicle](double t) { return vehicle.PredictStateAt(t); }, TRAJECTORY_STEPS);
      vehicle.UpdateTrajectory(trajectory);
    }
  }
}

void PathPlanning::PathPlanner::UpdatePlan(double processing_time) {
  LOG(INFO) << "Updating Next Plan (Processing Time: " << processing_time << " s)";
  FTrajectory next_trajectory =
      this->behaviour_planner.UpdatePlan(this->ego, this->traffic, TRAJECTORY_STEPS, processing_time);
  if (!next_trajectory.empty()) {
    this->ego.UpdateTrajectory(next_trajectory);
  }
}

PathPlanning::CTrajectory PathPlanning::PathPlanner::GetTrajectory() const {
  return trajectory_generator.FrenetToCartesian(this->ego.trajectory);
}
