#include "vehicle.h"

#include <iomanip>
#include <iostream>
#include "map.h"
#include "utils.h"

PathPlanning::Vehicle::Vehicle() : id(EGO_ID), s(0), s_v(0), s_a(0), d(0), d_v(0), d_a(0) {}
PathPlanning::Vehicle::Vehicle(int id, double s, double d) : id(id), s(s), s_v(0), s_a(0), d(d), d_v(0), d_a(0) {
  this->lane = Map::LaneIndex(d);
}

void PathPlanning::Vehicle::UpdatePosition(double s, double d) {
  this->s = Map::WrapDistance(s);
  this->d = d;
  this->lane = Map::LaneIndex(this->d);
}

void PathPlanning::Vehicle::UpdateVelocity(double s_v, double d_v) {
  this->s_v = s_v;
  this->d_v = d_v;
}

void PathPlanning::Vehicle::UpdateAcceleration(double s_a, double d_a) {
  this->s_a = s_a;
  this->d_a = d_a;
}