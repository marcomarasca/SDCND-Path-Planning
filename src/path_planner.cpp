#include "path_planner.h"
#include <set>
#include "utils.h"

PathPlanning::PathPlanner::PathPlanner(Map &map)
    : map(map), lanes_traffic(LANES_N), trajectory_generator(TRAJECTORY_STEP_DT) {}

void PathPlanning::PathPlanner::Update(const json &telemetry) {
  this->UpdateEgo(telemetry);
  this->UpdateTraffic(telemetry);
  this->UpdateTrajectory();
}

void PathPlanning::PathPlanner::UpdateEgo(const json &telemetry) {
  const std::size_t steps_to_go = telemetry["previous_path_x"].size();
  const std::size_t steps_consumed = this->f_trajectory.size() - steps_to_go;

  State s{telemetry["s"], 0.0, 0.0};
  State d{telemetry["d"], 0.0, 0.0};

  std::cout << "Prev Path: " << steps_to_go << std::endl;
  std::cout << "Steps Consumed: " << steps_consumed << std::endl;
  std::cout << "Telemetry: (S: " << s.p << ", D: " << d.p << ", Speed: " << Mph2ms(telemetry["speed"]) << ")"
            << std::endl;

  // If there is a previous trajectory uses the s and d computed previously
  if (steps_consumed > 0) {
    size_t trajectory_idx = steps_consumed - 1;
    std::cout << "Trajectory Index: " << trajectory_idx << std::endl;
    Frenet trajectory_step = this->f_trajectory[trajectory_idx];

    s = trajectory_step.s;
    d = trajectory_step.d;

    // compares with computed values

    /*     // Computes velocity and accelerations components from previous paths
        double prev_s, prev_d;
        double cs_v, cs_a, cd_v, cd_a;

        if (trajectory_idx == 0) {
          prev_s = this->ego.s;
          prev_d = this->ego.d;
          cs_v = (s - prev_s) / TRAJECTORY_STEP_DT;
          cd_v = (d - prev_d) / TRAJECTORY_STEP_DT;
          cs_a = (cs_v - this->ego.s_v) / TRAJECTORY_STEP_DT;
          cd_a = (cd_v - this->ego.d_v) / TRAJECTORY_STEP_DT;
        } else {
          prev_s = this->f_trajectory[trajectory_idx - 1].s[0];
          prev_d = this->f_trajectory[trajectory_idx - 1].d[0];
          cs_v = (s - prev_s) / TRAJECTORY_STEP_DT;
          cd_v = (d - prev_d) / TRAJECTORY_STEP_DT;
          cs_a = (cs_v - this->ego.s_v) / (steps_consumed * TRAJECTORY_STEP_DT);
          cd_a = (cd_v - this->ego.d_v) / (steps_consumed * TRAJECTORY_STEP_DT);
        }

        std::cout<<"From Trajectory:"<<std::endl;
        std::cout<<"S: "<< "(" <<s<<", "<<s_v<<", "<<s_a<<")"<<std::endl;
        std::cout<<"D: "<< "(" <<d<<", "<<d_v<<", "<<d_a<<")"<<std::endl;
        std::cout<<"Computed:"<<std::endl;
        std::cout<<"S: "<< "(" <<s<<", "<<cs_v<<", "<<cs_a<<")"<<std::endl;
        std::cout<<"D: "<< "(" <<d<<", "<<cd_v<<", "<<cd_a<<")"<<std::endl;

        s_v = cs_v;
        s_a = cs_a;
        d_v = cd_v;
        d_a = cd_a; */
  }

  this->ego.UpdateState(s, d);

  // this->ego.UpdatePosition(s_p, d_p);
  // this->ego.UpdateVelocity(s_v, d_v);
  // this->ego.UpdateAcceleration(s_a, d_a);
}

void PathPlanning::PathPlanner::UpdateTraffic(const json &telemetry) {
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  std::vector<std::set<size_t>> sensed_vehicles{3};
  // Reads sensor fusion data and maps it to the current vehicles
  // id, x, y, vx, vy, s, d,
  for (std::vector<double> vehicle_telemetry : sensor_fusion) {
    size_t id = vehicle_telemetry[0];
    double v_x = vehicle_telemetry[3];
    double v_y = vehicle_telemetry[4];
    double s_p = vehicle_telemetry[5];
    double s_v = Distance(0, v_y, v_x, 0);
    double d_p = vehicle_telemetry[6];

    if (d_p < 0) {  // Vehicle not on the rendered yet
      continue;
    }

    if (std::abs(s_p - this->ego.s.p) > SENSOR_RADIUS) {  // Vehicle out of sensor reach
      continue;
    }

    State s{s_p, s_v, 0.0};
    State d{d_p, 0.0, 0.0};

    size_t lane = Map::LaneIndex(d.p);

    auto &lane_traffic = this->lanes_traffic[lane];

    auto it = lane_traffic.find(id);

    if (it == lane_traffic.end()) {  // new vehicle
      Vehicle vehicle{id, s, d};
      lane_traffic.emplace(id, vehicle);
    } else {
      it->second.UpdateState(s, d);
    }
    sensed_vehicles[lane].emplace(id);
  }

  // Clean up the vehicles out of reach
  for (size_t lane = 0; lane < this->lanes_traffic.size(); ++lane) {
    auto &lane_traffic = this->lanes_traffic[lane];
    for (auto it = lane_traffic.cbegin(); it != lane_traffic.cend();) {
      size_t id = it->first;
      if (sensed_vehicles[lane].find(id) == sensed_vehicles[lane].end()) {
        it = lane_traffic.erase(it);
      } else {
        ++it;
      }
    }
  }

  for (size_t i = 0; i < this->lanes_traffic.size(); ++i) {
    std::cout << "Lane " << i << " traffic: " << lanes_traffic[i].size() << std::endl;
  }
}

void PathPlanning::PathPlanner::UpdateTrajectory() {
  // TODO behavior planning
  PathPlanning::Frenet start = {this->ego.s, this->ego.d};

  PathPlanning::Frenet target = ComputeTarget(start, ego.lane);

  std::cout << "Start:" << std::endl;
  std::cout << "S: " << start.s.p << " - " << start.s.v << " - " << start.s.a << std::endl;
  std::cout << "D: " << start.d.p << " - " << start.d.v << " - " << start.d.a << std::endl;

  std::cout << "Target:" << std::endl;
  std::cout << "S: " << target.s.p << " - " << target.s.v << " - " << target.s.a << std::endl;
  std::cout << "D: " << target.d.p << " - " << target.d.v << " - " << target.d.a << std::endl;

  this->f_trajectory = this->trajectory_generator.Generate(start, target, TRAJECTORY_STEPS);

  std::cout << std::endl;
}

PathPlanning::Frenet PathPlanning::PathPlanner::ComputeTarget(PathPlanning::Frenet &start, int target_lane) {
  // Velocity: v1 + a * t
  double s_v = std::min(MAX_SPEED, start.s.v + MAX_ACC * TRAJECTORT_T);
  // Acceleration: (v2 - v1) / t
  double projected_a = (s_v - start.s.v) / TRAJECTORT_T;
  // Constant acceleration: s1 + v1 * t + 0.5 * a * t^2
  double s_p = Map::WrapDistance(start.s.p + start.s.v * TRAJECTORT_T + 0.5 * projected_a * std::pow(TRAJECTORT_T, 2));
  double s_a = 0.0;

  double d_p = Map::LaneDisplacement(target_lane);
  double d_v = 0.0;
  double d_a = 0.0;

  State s{s_p, s_v, s_a};
  State p{d_p, d_v, d_a};

  return {s, p};
}

PathPlanning::Trajectory PathPlanning::PathPlanner::getGlobalCoordTrajectory() {
  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  for (auto step : this->f_trajectory) {
    double s = step.s.p;
    double d = step.d.p;
    auto coord = this->map.FrenetToCartesian(s, d);

    next_x_vals.emplace_back(coord[0]);
    next_y_vals.emplace_back(coord[1]);
  }

  return Trajectory({next_x_vals, next_y_vals});
}
