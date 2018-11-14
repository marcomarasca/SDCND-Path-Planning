#ifndef PP_BEHAVIOR_PLANNER_H
#define PP_BEHAVIOR_PLANNER_H

#include "map.h"
#include "vehicle.h"

namespace PathPlanning {

enum PlanState { KEEP_LANE, CHANGE_LANE_LEFT, CHANGE_LANE_RIGHT };

class BehaviorPlanner {
 private:
  PlanState state;
  Map &map;
  Vehicle &ego;

 public:
  BehaviorPlanner(Map &map, Vehicle &ego);
  ~BehaviorPlanner(){};

  PlanState NextPlan(double t);
};

}  // namespace PathPlanning

#endif