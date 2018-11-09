#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>

namespace PathPlanning {
// The max s value before wrapping around the track back to 0
const double WRAP_AROUND_S = 6945.554;

class Map {
 private:
  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;

 public:
  Map();
  ~Map(){};
  void LoadWaypoints(std::string file_path);

 private:
  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  std::vector<double> CartesianToFrenet(double x, double y, double theta);
  std::vector<double> FrenetToCartesian(double s, double d);
};

}  // namespace PathPlanning

#endif