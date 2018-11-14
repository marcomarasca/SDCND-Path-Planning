#include "path_planner.h"
#include <set>
#include "utils.h"

PathPlanning::PathPlanner::PathPlanner(Map &map)
    : map(map), behavior_planner(map, this->ego), trajectory_generator(TRAJECTORY_STEP_DT) {}

void PathPlanning::PathPlanner::Update(const json &telemetry) {
  this->UpdateEgo(telemetry);
  this->UpdateVehicles(telemetry);
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

void PathPlanning::PathPlanner::UpdateVehicles(const json &telemetry) {
  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry["sensor_fusion"];

  std::set<int> sensed_vehicles;
  // Reads sensor fusion data and maps it to the current vehicles
  // id, x, y, vx, vy, s, d,
  for (std::vector<double> vehicle_telemetry : sensor_fusion) {
    size_t id = vehicle_telemetry[0];
    double s_p = vehicle_telemetry[5];
    double d_p = vehicle_telemetry[6];

    if (d_p < 0) {  // Vehicle not on the rendered yet
      continue;
    }

    // TODO velocity?
    State s{s_p, 0.0, 0.0};
    State d{d_p, 0.0, 0.0};

    auto it = this->vehicles.find(id);

    if (it == this->vehicles.end()) { // new vehicle
      Vehicle vehicle{id, s, d};
      this->vehicles.emplace(id, vehicle);
    } else {
      it->second.UpdateState(s, d);
    }

    sensed_vehicles.emplace(id);
  }

  // Removes vehicles that are not sensed anymore
  for (auto it = this->vehicles.cbegin(); it != this->vehicles.cend();) {
    int id = it->first;
    if (sensed_vehicles.find(id) == sensed_vehicles.end()) {
      it = this->vehicles.erase(it);
    } else {
      ++it;
    }
  }
  std::cout << "Sensed Vehicles: " << this->vehicles.size() << std::endl;
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
