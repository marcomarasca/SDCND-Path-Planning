#ifndef PP_MAP_H
#define PP_MAP_H

#include <string>
#include <vector>
#include "spline.h"
#include "utils.h"

namespace PathPlanning {

struct Coord {
  double x;
  double y;
};

// The max s value before wrapping around the track back to 0
const double MAP_MAX_S = 6945.554;
// Lane width in meters
const double LANE_WIDTH = 4.0;
// Number of availble lanes
const size_t LANES_N = 3;

class Map {
 private:
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;

  double RoadAngle(double s);

 public:
  static size_t LaneIndex(double d);
  static double LaneDisplacement(size_t lane_index);
  static double WrapDistance(double s);

  Map();
  ~Map(){};
  void LoadWaypoints(std::string file_path);
  Coord FrenetToCartesian(double s, double d);
  std::pair<double, double> FrenetVelocity(double s, double d, double v_x, double v_y);
};

}  // namespace PathPlanning

#endif