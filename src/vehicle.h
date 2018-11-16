#ifndef PP_VEHICLE_H
#define PP_VEHICLE_H

#include "utils.h"

namespace PathPlanning {

const size_t EGO_ID = -1;

// Vehicle length in meters
const double VEHICLE_LENGTH = 5.0;

class Vehicle {
 public:
  static bool SComparator(const Vehicle &first, const Vehicle &second) { return first.state.s.p < second.state.s.p; };

  size_t id;
  Frenet state;
  FTrajectory trajectory;
  size_t lane;

  Vehicle(size_t id = EGO_ID);
  Vehicle(size_t id, const Frenet &state);
  ~Vehicle(){};

  void UpdateState(const Frenet &state);
  void UpdateTrajectory(const FTrajectory &trajectory);
  void PredictTrajectory(size_t steps, double step_dt);
  Frenet StateAt(double t) const;
};

}  // namespace PathPlanning

#endif