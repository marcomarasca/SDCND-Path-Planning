#ifndef VEHICLE_H
#define VEHICLE_H

namespace PathPlanning {

const int EGO_ID = -1;

class Vehicle {
 public:
  int id;
  double s;  // Longitudinal displacement
  double s_v;
  double s_a;
  double d;  // Lateral displacement
  double d_v;
  double d_a;
  int lane;

 public:
  Vehicle();
  Vehicle(int id, double s, double d);
  ~Vehicle(){};

  void UpdatePosition(double s, double d);
  void UpdateVelocity(double s_v, double d_v);
  void UpdateAcceleration(double s_a, double d_a);
};

}  // namespace PathPlanning

#endif