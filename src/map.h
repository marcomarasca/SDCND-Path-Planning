#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>

namespace PathPlanning {
// The max s value before wrapping around the track back to 0
const double MAP_MAX_S = 6945.554;
// Lane width in meters
const double LANE_WIDTH = 4.0;

class Map {
 private:
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;

 public:
  static int LaneIndex(double d);

  Map();
  ~Map(){};
  void LoadWaypoints(std::string file_path);
  std::vector<double> CartesianToFrenet(double x, double y, double theta);
  std::vector<double> FrenetToCartesian(double s, double d);

 private:
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
};

}  // namespace PathPlanning

#endif