#ifndef PP_VEHICLE_H
#define PP_VEHICLE_H

#include "utils.h"

namespace PathPlanning {

const size_t EGO_ID = -1;

class Vehicle {
 public:
  size_t id;
  State s;
  State d;
  size_t lane;

 public:
  Vehicle(size_t id = EGO_ID);
  Vehicle(size_t id, const State &s, const State &d);
  ~Vehicle(){};

  void UpdateState(const State &s, const State &d);
};

}  // namespace PathPlanning

#endif