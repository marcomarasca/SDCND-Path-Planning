#include "behavior_planner.h"

PathPlanning::BehaviorPlanner::BehaviorPlanner(Map &map, Vehicle &ego)
    : state(PlanState::KEEP_LANE), map(map), ego(ego) {}

PathPlanning::PlanState PathPlanning::BehaviorPlanner::NextPlan(double t) { return PathPlanning::PlanState::KEEP_LANE; }