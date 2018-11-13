#ifndef VEHICLE_H
#define VEHICLE_H

namespace PathPlanning {

const int EGO_ID = -1;

class Vehicle {
 private:
  int id;
  double x;
  double y;
  double s;
  double d;
  double speed;
  double yaw;
  int lane;

 public:
  Vehicle();
  Vehicle(int id, double x, double y, double s, double d, double speed, double yaw);
  ~Vehicle(){};

  void Update(double x, double y, double s, double d, double speed, double yaw);
  void PrintState();
};

}  // namespace PathPlanning

#endif