#include "path_planner.h"
#include <algorithm>
#include <map>
#include "logger.h"
#include "utils.h"

PathPlanning::PathPlanner::PathPlanner(Map &map, size_t lane_n)
    : map(map),
      ego(EGO_ID),
      traffic(lane_n),
      trajectory_generator(map, TRAJECTORY_STEP_DT, MAX_SPEED, MAX_ACC),
      behaviour_planner(this->trajectory_generator, MAX_SPEED, MIN_SPEED, MAX_ACC) {}

void PathPlanning::PathPlanner::Update(const json &telemetry, double processing_time) {
  LOG(INFO) << "Updating Path Planner (Processing time: " << processing_time << " s)";

  this->UpdateEgo(telemetry);
  this->UpdateTraffic(telemetry);
  this->UpdatePredictions();
  this->UpdatePlan(std::max(processing_time, MIN_PROCESSING_TIME));
}

void PathPlanning::PathPlanner::UpdateEgo(const json &telemetry) {
  const int steps_to_go = telemetry["previous_path_x"].size();
  const int steps_consumed = this->ego.trajectory.size() - steps_to_go;

  const double s_p = telemetry["s"];
  const double s_v = Mph2ms(telemetry["speed"]);
  const double d_p = telemetry["d"];

  Frenet state{{s_p, s_v, 0.0}, {d_p, 0.0, 0.0}};

  const size_t col_w = 12;

  LOG(INFO) << LOG_P << "Updating Vehicle State (Consumed: " << steps_consumed << ", Unvisited: " << steps_to_go
            << "): ";
  LOG(INFO) << LOG_P1 << std::left << std::setw(col_w) << "Telemetry: " << std::setw(col_w) << "s_p" << std::setw(col_w)
            << "d_p" << std::setw(col_w) << "s_v" << std::setw(col_w) << "d_v";
  LOG(INFO) << LOG_P1 << std::left << std::setw(col_w) << "Received: " << std::setw(col_w) << state.s.p
            << std::setw(col_w) << state.d.p << std::setw(col_w) << state.s.v << std::setw(col_w) << state.d.v;

  this->ego.UpdateState(state);

  if (steps_to_go == 0) {
    // Reset plan (keeps current state as target)
    this->behaviour_planner.ResetPlan(state);
    this->ego.ResetTrajectory();
  } else if (steps_consumed > 0) {
    // Use data from the previous trajectory
    this->ego.ForwardState(steps_consumed - 1);
  }

  LOG(INFO) << LOG_P1 << std::left << std::setw(col_w) << "Using: " << std::setw(col_w) << this->ego.state.s.p
            << std::setw(col_w) << this->ego.state.d.p << std::setw(col_w) << this->ego.state.s.v << std::setw(col_w)
            << this->ego.state.d.v;
}

void PathPlanning::PathPlanner::UpdateTraffic(const json &telemetry) {
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  LOG(INFO) << LOG_P << "Updating Traffic Data (Sensed Vehicles: " << sensor_fusion.size() << ")";

  // Keeps old data
  if (sensor_fusion.empty()) {
    LOG(DEBUG) << LOG_P1 << "Invalid Sensor Data (Empty List)";
    return;
  }

  std::map<int, Pair> vehicles;

  for (auto &lane : this->traffic) {
    // Maps vehicles id to their frenet coordinates
    std::for_each(lane.begin(), lane.end(), [&vehicles](const Vehicle &v) {
      Pair coord{v.state.s.p, v.state.d.p};
      vehicles.emplace(v.id, coord);
    });
    // Reserve some space in the lane
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
      LOG(DEBUG) << LOG_P1 << "Invalid Displacement for Vehicle " << id << " (Displacement: " << d_p << ")";
      continue;
    }

    Pair prev{s_p, d_p};

    auto it = vehicles.find(id);

    if (it != vehicles.end()) {
      prev = it->second;
    }

    // Accounts for invalid data sent by the simulator when the maps wraps around

    if (Map::ModDistance(s_p, prev.first) < 0) {  // Vehicles do not go back in time
      LOG(WARN) << LOG_P1 << "Invalid S for Vehicle " << id << " (Received: " << s_p << ", Previous: " << prev.first
                << ")";
      s_p = prev.first;
    }

    if (std::fabs(d_p - prev.second) > LANE_WIDTH) {  // Vehicles cannot move right/left too much
      LOG(WARN) << LOG_P1 << "Invalid D for Vehicle " << id << " (Received: " << d_p << ", Previous: " << prev.second
                << ")";
      d_p = prev.second;
    }

    size_t lane = Map::LaneIndex(d_p);

    if (Map::InvalidLane(lane)) {
      LOG(WARN) << LOG_P1 << "Invalid Lane for Vehicle " << id << " (Lane: " << lane << ")";
      continue;
    }

    if (std::fabs(Map::ModDistance(s_p, this->ego.state.s.p)) > RANGE) {  // Vehicle out of sensor reach
      continue;
    }

    double v_x = vehicle_telemetry[3];
    double v_y = vehicle_telemetry[4];

    auto frenet_v = this->map.FrenetVelocity(s_p, d_p, v_x, v_y);

    const Frenet state{{s_p, frenet_v.first, 0.0}, {d_p, frenet_v.second, 0.0}};

    this->traffic[lane].emplace_back(id, state);
  }

  const double s = this->ego.state.s.p;
  size_t lane = 0;

  for (auto &lane_traffic : this->traffic) {
    LOG(DEBUG) << LOG_P1 << "Lane " << lane << " Traffic: " << lane_traffic.size();
    // Sort the vehicles by distance to the ego vehicle
    std::sort(lane_traffic.begin(), lane_traffic.end(), [&s](const Vehicle &a, const Vehicle &b) {
      return Map::ModDistance(a.state.s.p, s) < Map::ModDistance(b.state.s.p, s);
    });
    ++lane;
  }
}

void PathPlanning::PathPlanner::UpdatePredictions() {
  LOG(INFO) << LOG_P << "Updating Traffic Predictions";
  const size_t trajectory_length = this->trajectory_generator.TrajectoryLength(TRAJECTORY_T);
  for (auto &lane_traffic : this->traffic) {
    for (auto &vehicle : lane_traffic) {
      auto predict_state_fn = [&vehicle](double t) { return vehicle.PredictStateAt(t); };
      FTrajectory trajectory = this->trajectory_generator.Predict(vehicle.state, predict_state_fn, trajectory_length);
      vehicle.UpdateTrajectory(trajectory);
    }
  }
}

void PathPlanning::PathPlanner::UpdatePlan(double processing_time) {
  LOG(INFO) << LOG_P << "Updating Plan (Processing Time: " << processing_time << " s)";
  Plan plan = this->behaviour_planner.Update(this->ego, this->traffic, TRAJECTORY_T, processing_time);
  if (!plan.trajectory.empty()) {
    this->ego.UpdateTrajectory(plan.trajectory);
  }
}

PathPlanning::CTrajectory PathPlanning::PathPlanner::GetTrajectory() const {
  return trajectory_generator.FrenetToCartesian(this->ego.trajectory);
}

void PathPlanning::PathPlanner::DrawRoad(double processing_time) {
  const size_t ego_lane = this->ego.GetLane();
  const double target_s_p = this->ego.trajectory.back().s.p;
  const size_t target_lane = Map::LaneIndex(this->ego.trajectory.back().d.p);

  // Move to top of the screen (*unix terminal only)
  std::cout << __TO_TOP__;
  std::cout << __CLEAR_LINE__ << "P: " << this->ego.state.s.p << " m" << '\n';
  std::cout << __CLEAR_LINE__ << "L: " << Map::LaneIndex(this->ego.trajectory.back().d.p) << '\n';
  std::cout << __CLEAR_LINE__ << "T: " << processing_time << " s\n\n";
  for (int i = DRAW_AHEAD; i > -DRAW_BEHIND; i -= VEHICLE_LENGTH) {
    const double ref_s = this->ego.state.s.p + i;
    size_t lane_n = 0;
    for (auto &lane_traffic : traffic) {
      if (lane_n == 0) {
        std::cout << __C_MID_LANE__ << "||";
      } else if (i % 2 != 0) {
        std::cout << "|";
      } else {
        std::cout << " ";
      }
      std::cout << __C_RESET__;
      bool empty_lane = true;
      for (auto &vehicle : lane_traffic) {
        if (std::fabs(Map::ModDistance(ref_s, vehicle.state.s.p)) <= VEHICLE_LENGTH / 2) {
          empty_lane = false;
          std::cout << __C_TRAFFIC__ << " " << std::setfill('0') << std::setw(2) << vehicle.id << " " << __C_RESET__;
          break;
        }
      }
      if (empty_lane) {
        if (lane_n == ego_lane && this->ego.state.s.p == ref_s) {
          std::cout << __C_EGO__ << "[<>]" << __C_RESET__;  // Ego vehicle
          double target_s_p = this->ego.trajectory.back().s.p;
        } else if (lane_n == target_lane && std::fabs(Map::ModDistance(ref_s, target_s_p)) <= VEHICLE_LENGTH / 2) {
          std::cout << __C_TARGET__ << "[><]" << __C_RESET__;  // Trajectory target
        } else {
          std::cout << "    ";  // Empty
        }
      }
      ++lane_n;
    }
    std::cout << "|" << __C_RESET__ << "\n";
  }
  std::cout << std::flush;
}
