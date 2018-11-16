#include "vehicle.h"

#include <iomanip>
#include <iostream>
#include "map.h"

PathPlanning::Vehicle::Vehicle(size_t id) : id(id), lane(0) {}

PathPlanning::Vehicle::Vehicle(size_t id, const State &s, const State &d) : id(id) { this->UpdateState(s, d); }

void PathPlanning::Vehicle::UpdateState(const State &s, const State &d) {
  this->s = s;
  this->d = d;
  this->lane = Map::LaneIndex(this->d.p);
}