#ifndef PP_MAP_H
#define PP_MAP_H

#include <string>
#include "spline.h"
#include "utils.h"

namespace PathPlanning {

// The max s value before wrapping around the track back to 0
const double MAP_MAX_S = 6945.554;
// Lane width in meters
const double LANE_WIDTH = 4.0;
// Number of availble lanes
const size_t LANES_N = 3;
// Bugfix for simulator bug
const double MAP_MAX_D = 9.9;

using Pair = std::pair<double, double>;

/**
 * The class is used to handle localization information about the vehicles on the road
 */
class Map {
 private:
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;

 public:
  /**
   * Computes the lane index (from 0 to 2) given the displacement
   */
  static size_t LaneIndex(double d);
  /**
   * Computes the lane displacement d for the lane with the given index
   */
  static double LaneDisplacement(size_t lane_index);
  /**
   * Checks if the given lane index is invalid (e.g. out of range)
   */
  static bool InvalidLane(size_t lane_index);
  /**
   * Helper method to wrap around the circular map when reaching past the latest waypoint
   */
  static double Mod(double s);
  /**
   * Computes the distance between the given longitudinal frenet components taking into account the wrapping of the map
   * (e.g. useful when s1 is past the latest waypoint and s2 is prior but they are still in within half of the map
   * reach)
   */
  static double ModDistance(double s1, double s2);

  Map();
  ~Map(){};

  /**
   * Loads the map waypoints from the given file
   */
  void LoadWaypoints(std::string file_path);

  /**
   * Transform from Frenet s,d coordinates to Cartesian x,y
   */
  Pair FrenetToCartesian(double s, double d) const;

  /**
   * Computes the s and d velocity components from the given frenet coordinates and velocity vector component (in
   * cartesian coordinates)
   */
  Pair FrenetVelocity(double s, double d, double v_x, double v_y) const;

 private:
  /**
   * Returns the road angle at the given s
   */
  double RoadAngle(double s) const;
};

}  // namespace PathPlanning

#endif