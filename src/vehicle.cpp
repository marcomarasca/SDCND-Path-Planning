#include "vehicle.h"
#include "map.h"

PathPlanning::Vehicle::Vehicle(size_t id) : id(id), lane(0) {}

PathPlanning::Vehicle::Vehicle(size_t id, const Frenet &state) : id(id) { this->UpdateState(state); }

void PathPlanning::Vehicle::UpdateState(const Frenet &state) {
  this->state = state;
  this->lane = Map::LaneIndex(this->state.d.p);
}

void PathPlanning::Vehicle::UpdateTrajectory(const PathPlanning::FTrajectory &trajectory) {
  this->trajectory = trajectory;
}

void PathPlanning::Vehicle::PredictTrajectory(size_t steps, double step_dt) {
  this->trajectory.clear();
  for (size_t i = 1; i <= steps; ++i) {
    const double t = i * step_dt;
    this->trajectory.emplace_back(this->StateAt(t));
  }
}

PathPlanning::Frenet PathPlanning::Vehicle::StateAt(double t) const {
  const double t_2 = t * t;
  const double s_p = this->state.s.p + this->state.s.v * t + 0.5 * this->state.s.a * t_2;
  const double s_v = this->state.s.v + this->state.s.a * t;
  const double d_p = this->state.d.p + this->state.d.v * t + 0.5 * this->state.d.a * t_2;
  const double d_v = this->state.d.v + this->state.d.a * t;
  return {{s_p, this->state.s.v, this->state.s.a}, {d_p, this->state.d.v, this->state.d.a}};
}