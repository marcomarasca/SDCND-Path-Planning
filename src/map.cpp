#include "map.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "logger.h"

size_t PathPlanning::Map::LaneIndex(double d) { return size_t(d / PathPlanning::LANE_WIDTH); }
double PathPlanning::Map::LaneDisplacement(size_t lane_index) { return std::min(lane_index * LANE_WIDTH + LANE_WIDTH / 2.0, MAP_MAX_D); }
bool PathPlanning::Map::InvalidLane(size_t lane_index) { return lane_index >= LANES_N; };
double PathPlanning::Map::Mod(double s) { return std::fmod(s, MAP_MAX_S); }
double PathPlanning::Map::ModDistance(double s1, double s2) {
  // Checks the correct wrapping when comparing distances that are on different side of the circuit
  // E.g. 4--5--6--0--1--2--3 |-->4
  // If a s1 is 1 and s2 is 6, then distance should be -2 since the map wraps at 7
  double distance = Mod(s1) - Mod(s2);
  if (distance < -MAP_MAX_S / 2.0) {
    distance += MAP_MAX_S;
  }
  if (distance > MAP_MAX_S / 2.0) {
    distance -= MAP_MAX_S;
  }

  return distance;
}

PathPlanning::Map::Map() {}

void PathPlanning::Map::LoadWaypoints(std::string file_path) {
  LOG(INFO) << "Loading map waypoints from: " << file_path;

  std::ifstream map_file(file_path.c_str(), std::ifstream::in);

  if (!map_file) {
    throw std::runtime_error("Cannot not open file: " + file_path);
  }

  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;

  double x, y, s, dx, dy;

  std::string line;
  while (std::getline(map_file, line)) {
    std::istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    waypoints_x.emplace_back(x);
    waypoints_y.emplace_back(y);
    waypoints_s.emplace_back(s);
    waypoints_dx.emplace_back(dx);
    waypoints_dy.emplace_back(dy);
  }

  map_file.close();

  waypoints_x.emplace_back(waypoints_x[0]);
  waypoints_y.emplace_back(waypoints_y[0]);
  waypoints_s.emplace_back(MAP_MAX_S);
  waypoints_dx.emplace_back(waypoints_dx[0]);
  waypoints_dy.emplace_back(waypoints_dy[0]);

  // Build the splines to get a smooth transition
  this->spline_x.set_points(waypoints_s, waypoints_x);
  this->spline_y.set_points(waypoints_s, waypoints_y);
  this->spline_dx.set_points(waypoints_s, waypoints_dx);
  this->spline_dy.set_points(waypoints_s, waypoints_dy);
}

PathPlanning::Pair PathPlanning::Map::FrenetToCartesian(double s, double d) const {
  s = Mod(s);
  const double x = this->spline_x(s) + this->spline_dx(s) * d;
  const double y = this->spline_y(s) + this->spline_dy(s) * d;

  return {x, y};
}

PathPlanning::Pair PathPlanning::Map::FrenetVelocity(double s, double d, double v_x, double v_y) const {
  const double road_angle = this->RoadAngle(s);
  const double v_angle = Angle(v_x, v_y);
  const double v = Distance(0, 0, v_x, v_y);
  const double angle_diff = v_angle - road_angle;

  const double s_v = std::fabs(v * std::cos(angle_diff));
  const double d_v = v * std::sin(angle_diff);

  return {s_v, d_v};
}

double PathPlanning::Map::RoadAngle(double s) const {
  s = Mod(s);
  // This is equivalent to take a small s increment and compute the delta
  const double d_x = this->spline_x.deriv(1, s);
  const double d_y = this->spline_y.deriv(1, s);

  return Angle(d_x, d_y);
}
