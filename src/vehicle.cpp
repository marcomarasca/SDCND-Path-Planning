#include "vehicle.h"
#include "map.h"

PathPlanning::Vehicle::Vehicle(int id) : id(id) {}

PathPlanning::Vehicle::Vehicle(int id, const Frenet &state) : id(id), state(state) { this->PositionUpdated(); }

size_t PathPlanning::Vehicle::GetLane() const { return Map::LaneIndex(this->state.d.p); }

PathPlanning::Frenet PathPlanning::Vehicle::StateAt(size_t trajectory_step) const {
  assert(trajectory_step < this->trajectory.size());
  return this->trajectory[trajectory_step];
}

void PathPlanning::Vehicle::UpdateState(const Frenet &state) {
  this->state = state;
  this->PositionUpdated();
}

void PathPlanning::Vehicle::UpdateTrajectory(const PathPlanning::FTrajectory &trajectory) {
  this->trajectory = trajectory;
}

void PathPlanning::Vehicle::ForwardState(size_t trajectory_step) {
  assert(trajectory_step < this->trajectory.size());
  this->state = this->trajectory[trajectory_step];
  this->trajectory.erase(this->trajectory.begin(), this->trajectory.begin() + trajectory_step + 1);
}

void PathPlanning::Vehicle::ResetTrajectory() { this->trajectory.clear(); }

void PathPlanning::Vehicle::PositionUpdated() { this->state.s.p = Map::Mod(this->state.s.p); }

PathPlanning::Frenet PathPlanning::Vehicle::PredictStateAt(double t) const {
  // Simple constant acceleration model
  const double t_2 = t * t;
  const double s_p = this->state.s.p + this->state.s.v * t + 0.5 * this->state.s.a * t_2;
  const double s_v = this->state.s.v + this->state.s.a * t;
  const double d_p = this->state.d.p + this->state.d.v * t + 0.5 * this->state.d.a * t_2;
  const double d_v = this->state.d.v + this->state.d.a * t;
  return {{s_p, this->state.s.v, this->state.s.a}, {d_p, this->state.d.v, this->state.d.a}};
}