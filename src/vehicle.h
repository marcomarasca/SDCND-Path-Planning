#ifndef PP_VEHICLE_H
#define PP_VEHICLE_H

#include "utils.h"

namespace PathPlanning {

// Vehicle length in meters
const double VEHICLE_LENGTH = 5.0;

class Vehicle {
 public:
  int id;
  Frenet state;
  FTrajectory trajectory;

  Vehicle(int id);
  Vehicle(int id, const Frenet &state);
  ~Vehicle(){};

  Frenet StateAt(double t) const;
  size_t GetLane() const;

  void UpdateState(const Frenet &state);
  void UpdateTrajectory(const FTrajectory &trajectory);
  void PredictTrajectory(size_t steps, double step_dt);
};

}  // namespace PathPlanning

#endif