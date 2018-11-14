#ifndef MAP_H
#define MAP_H

#include <string>
#include <vector>
#include "spline.h"
#include "utils.h"

namespace PathPlanning {

using tk::spline;

// The max s value before wrapping around the track back to 0
const double MAP_MAX_S = 6945.554;
// Lane width in meters
const double LANE_WIDTH = 4.0;
// Max speed in m/s
const double MAX_SPEED = Mph2ms(50);

class Map {
 private:
  spline spline_x;
  spline spline_y;
  spline spline_dx;
  spline spline_dy;

 public:
  static int LaneIndex(double d);
  static double WrapS(double s);

  Map();
  ~Map(){};
  void LoadWaypoints(std::string file_path);
  std::vector<double> FrenetToCartesian(double s, double d);
};

}  // namespace PathPlanning

#endif