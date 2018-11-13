#ifndef PLANNER_H
#define PLANNER_H

#include <map>
#include <vector>
#include "json.hpp"
#include "map.h"
#include "vehicle.h"

namespace PathPlanning {

enum VehicleState {

};

using json = nlohmann::json;

using Path = std::vector<std::vector<double>>;

class PathPlanner {
 private:
  Map map;
  Vehicle ego;
  std::map<int, Vehicle> vehicles;
  Path current_path;

 public:
  PathPlanner(const Map &map);
  ~PathPlanner(){};

  void Update(const json &telemetry);
  Path NextPath();

 private:
  void ParseTelemetry(const json &telemetry);
};

}  // namespace PathPlanning

#endif