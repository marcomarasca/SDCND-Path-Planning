#include "map.h"

#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>

size_t PathPlanning::Map::LaneIndex(double d) { return (int)(d / PathPlanning::LANE_WIDTH); }
double PathPlanning::Map::LaneDisplacement(size_t lane_index) { return lane_index * LANE_WIDTH + LANE_WIDTH / 2; }
double PathPlanning::Map::WrapDistance(double s) { return fmod(s, MAP_MAX_S); }

PathPlanning::Map::Map() {}

void PathPlanning::Map::LoadWaypoints(std::string file_path) {
  std::cout << "Loading map waypoints from: " << file_path << std::endl;

  std::ifstream map_file(file_path.c_str(), std::ifstream::in);

  if (!map_file) {
    throw std::runtime_error("Cannot not open file: " + file_path);
  }

  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;

  double saveFirstLine = true;
  double x0, y0, dx0, dy0;
  double x, y, s, dx, dy;

  std::string line;
  while (std::getline(map_file, line)) {
    std::istringstream iss(line);

    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dx;
    iss >> dy;

    if (saveFirstLine) {
      x0 = x;
      y0 = y;
      dx0 = dx;
      dy0 = dy;
      saveFirstLine = false;
    }

    waypoints_x.emplace_back(x);
    waypoints_y.emplace_back(y);
    waypoints_s.emplace_back(s);
    waypoints_dx.emplace_back(dx);
    waypoints_dy.emplace_back(dy);
  }

  map_file.close();

  waypoints_x.emplace_back(x0);
  waypoints_y.emplace_back(y0);
  waypoints_s.emplace_back(MAP_MAX_S);
  waypoints_dx.emplace_back(dx0);
  waypoints_dy.emplace_back(dy0);

  this->spline_x.set_points(waypoints_s, waypoints_x);
  this->spline_y.set_points(waypoints_s, waypoints_y);
  this->spline_dx.set_points(waypoints_s, waypoints_dx);
  this->spline_dy.set_points(waypoints_s, waypoints_dy);
}

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> PathPlanning::Map::FrenetToCartesian(double s, double d) {
  const double s_w = WrapDistance(s);

  const double x = this->spline_x(s_w) + this->spline_dx(s_w) * d;
  const double y = this->spline_y(s_w) + this->spline_dy(s_w) * d;

  return {x, y};
}
