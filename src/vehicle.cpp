#include "vehicle.h"
#include "map.h"

PathPlanning::Vehicle::Vehicle(int id) : id(id) {}

PathPlanning::Vehicle::Vehicle(int id, const Frenet &state) : id(id), state(state) { this->PositionUpdated(); }

size_t PathPlanning::Vehicle::GetLane() const { return Map::LaneIndex(this->state.d.p); }

PathPlanning::Frenet PathPlanning::Vehicle::StateAt(size_t trajectory_step) const {
  assert(trajectory_step >= 0 && trajectory_step < this->trajectory.size() - 1);
  return this->trajectory[trajectory_step];
}

void PathPlanning::Vehicle::UpdateState(const Frenet &state) {
  this->state = state;
  this->PositionUpdated();
}

void PathPlanning::Vehicle::UpdateTrajectory(const PathPlanning::FTrajectory &trajectory) {
  this->trajectory = trajectory;
}

void PathPlanning::Vehicle::PredictTrajectory(size_t steps, double step_dt) {
  this->trajectory.clear();
  this->trajectory.reserve(steps);
  for (size_t i = 1; i <= steps; ++i) {
    const double t = i * step_dt;
    this->trajectory.emplace_back(this->PredictStateAt(t));
  }
}

void PathPlanning::Vehicle::ResetTrajectory() {
  this->trajectory.clear();
}

void PathPlanning::Vehicle::PositionUpdated() { this->state.s.p = Map::Mod(this->state.s.p); }

PathPlanning::Frenet PathPlanning::Vehicle::PredictStateAt(double t) const {
  const double t_2 = t * t;
  const double s_p = this->state.s.p + this->state.s.v * t + 0.5 * this->state.s.a * t_2;
  const double s_v = this->state.s.v + this->state.s.a * t;
  const double d_p = this->state.d.p + this->state.d.v * t + 0.5 * this->state.d.a * t_2;
  const double d_v = this->state.d.v + this->state.d.a * t;
  return {{s_p, this->state.s.v, this->state.s.a}, {d_p, this->state.d.v, this->state.d.a}};
}