#include "vehicle.h"

#include <iomanip>
#include <iostream>
#include "map.h"
#include "utils.h"

PathPlanning::Vehicle::Vehicle() : id(EGO_ID), x(0), y(0), s(0), d(0), speed(0), yaw(0), lane(0) {}
PathPlanning::Vehicle::Vehicle(int id, double x, double y, double s, double d, double speed, double yaw)
    : id(id), x(x), y(y), s(s), d(d), speed(speed), yaw(yaw) {
  this->lane = Map::LaneIndex(d);
}

void PathPlanning::Vehicle::Update(double x, double y, double s, double d, double speed, double yaw) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->yaw = yaw;
  this->lane = Map::LaneIndex(this->d);
}

void PathPlanning::Vehicle::PrintState() {
  int h_fill = 13;
  std::cout << std::left << std::setw(h_fill) << "S";
  std::cout << std::left << std::setw(h_fill) << "| D";
  std::cout << std::left << std::setw(h_fill) << "| Speed";
  std::cout << std::left << std::setw(h_fill) << "| Yaw";
  std::cout << std::endl;
  std::cout << std::left << std::setw(h_fill) << this->s;
  std::cout << std::left << "| " << std::setw(h_fill - 2) << this->d;
  std::cout << std::left << "| " << std::setw(h_fill - 2) << this->speed;
  std::cout << std::left << "| " << std::setw(h_fill - 2) << this->yaw;
  std::cout << std::endl;
}