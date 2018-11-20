#ifndef PP_VEHICLE_H
#define PP_VEHICLE_H

#include "utils.h"

namespace PathPlanning {

// Vehicle length in meters
const double VEHICLE_LENGTH = 5.0;

/**
 * Holds the information about a vehicle on the road providing useful functions to manage their state
 */
class Vehicle {
 public:
  int id;
  Frenet state;
  // Current or predicted trajectory
  FTrajectory trajectory;

  Vehicle(int id);
  Vehicle(int id, const Frenet &state);
  ~Vehicle(){};

  /**
   * Returns the lane index of the vheicle according to its displacement
   */
  size_t GetLane() const;

  /**
   * Returns the state of the vehicle trajectory at the given step
   */
  Frenet StateAt(size_t trajectory_step) const;

  /**
   * Returns an estimate of the state at time t considering the current state of the vehicle
   */
  Frenet PredictStateAt(double t) const;

  /**
   * Updates the current state of the vehicle
   */
  void UpdateState(const Frenet &state);

  /**
   * Updates the vehicle trajectory
   */
  void UpdateTrajectory(const FTrajectory &trajectory);

  /**
   * Forward the current state to the given trajectory step, also erase the past trajectory steps
   */
  void ForwardState(size_t trajectory_step);

  /**
   * Resets the current trajectory
   */
  void ResetTrajectory();

 private:
  /**
   * Frenet state updated, makes sure the frenet coordinates are correct
   */
  void PositionUpdated();
};

}  // namespace PathPlanning

#endif