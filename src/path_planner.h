#ifndef PP_PATH_PLANNER_H
#define PP_PATH_PLANNER_H

#include "behaviour_planner.h"
#include "json.hpp"
#include "map.h"
#include "trajectory_generator.h"
#include "vehicle.h"

namespace PathPlanning {

// Id of the ego vehicle
const int EGO_ID = -1;
// Ego vehicle range in meters for vehicles detection (in front and behind)
const double RANGE = 100;
// Total time in seconds for the trajectory
const double TRAJECTORY_T = 2.0;
// Delta t between trajectory points in seconds (same as simulator controller update rate)
const double TRAJECTORY_STEP_DT = 0.02;
// Min number of second for the processing time to consider when including points from the past trajectory
const double MIN_PROCESSING_TIME = 5 * TRAJECTORY_STEP_DT;
// Max speed in m/s
const double MAX_SPEED = Mph2ms(47);
// Min speed in m/s
const double MIN_SPEED = Mph2ms(15);
// Max acceleration in m/s^2
const double MAX_ACC = 10.0;
// Number of meters ahead of the road to draw, must be multiple of vehicle length
const double DRAW_AHEAD = 135;
// Number of meters behind of the road to draw, must be multiple of vehicle length
const double DRAW_BEHIND = 50;

#if defined(__LINUX__) || defined(__gnu_linux__) || defined(__linux__)
#define __LX_DRAW__
#endif

// ANSI Drawing colors codes (https://en.wikipedia.org/wiki/ANSI_escape_code)
#ifdef __LX_DRAW__
#define __C_MID_LANE__ "\033[0;33m"
#define __C_TRAFFIC__ "\033[1;7;33m"
#define __C_EGO__ "\033[1;32m"
#define __C_TARGET__ "\033[1;36m"
#define __C_RESET__ "\033[0m"
#define __TO_TOP__ "\x1b[H"
#define __CLEAR_SCREEN__ "\x1b[H\x1b[J"
#define __CLEAR_LINE__ "                            \r"
#else
#define __C_MID_LANE__ ""
#define __C_TRAFFIC__ ""
#define __C_EGO__ ""
#define __C_TARGET__ ""
#define __C_RESET__ ""
#define __TO_TOP__ ""
#define __CLEAR_SCREEN__ ""
#define __CLEAR_LINE__ ""
#endif

using json = nlohmann::json;

/**
 * Main orchestrator for path planning
 */
class PathPlanner {
 private:
  const Map &map;
  Vehicle ego;
  Traffic traffic;
  TrajectoryGenerator trajectory_generator;
  BehaviourPlanner behaviour_planner;

 public:
  PathPlanner(Map &map, size_t lane_n = LANES_N);
  ~PathPlanner(){};

  /**
   * Updates the current plan according to the given telemetry and the estimated processing time (in seconds) to account
   * for delays in the controller
   */
  void Update(const json &telemetry, double processing_time);

  /**
   * Return the next trajectory in (global) cartesian coordinates
   */
  CTrajectory GetTrajectory() const;

  /**
   * Draws to standard output a representation of the current status
   */
  void DrawRoad(double processing_time);

 private:
  /**
   * Update the current state of the ego vehicle
   */
  void UpdateEgo(const json &telemetry);

  /**
   * Updates the state of the road traffic
   */
  void UpdateTraffic(const json &telemetry);

  /**
   * Updates the estimated predictions for the vehicles on the road
   */
  void UpdatePredictions();

  /**
   * Updates the plan according to the current state, accounts for eventual delays given by the estimated processing
   * time (in seconds)
   */
  void UpdatePlan(double processing_time);
};

}  // namespace PathPlanning

#endif