#ifndef PP_VEHICLE_H
#define PP_VEHICLE_H

#include "utils.h"

namespace PathPlanning {

// Vehicle length in meters
const double VEHICLE_LENGTH = 5.0;

class Vehicle {
 private:
  void PositionUpdated();

 public:
  int id;
  Frenet state;
  FTrajectory trajectory;

  Vehicle(int id);
  Vehicle(int id, const Frenet &state);
  ~Vehicle(){};

  size_t GetLane() const;
  Frenet StateAt(size_t trajectory_step) const;
  Frenet PredictStateAt(double t) const;

  void UpdateState(const Frenet &state);
  void UpdateTrajectory(const FTrajectory &trajectory);

  void ResetTrajectory();
};

}  // namespace PathPlanning

#endif