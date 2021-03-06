#ifndef PP_UTILS_H
#define PP_UTILS_H

#include <chrono>
#include <cmath>
#include <string>
#include <vector>

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

namespace PathPlanning {

const std::string LOG_P = "-> ";
const std::string LOG_P1 = " ";
const std::string LOG_P2 = "  * ";
const size_t COST_LOG_BUFFER = 22;
const size_t COST_LOG_WIDTH = 12;

struct State {
  // Position
  double p;
  // Velocity
  double v;
  // Acceleration
  double a;

  State() : p(0), v(0), a(0){};
  State(double p, double v, double a) : p(p), v(v), a(a){};
};

struct Frenet {
  State s;
  State d;
  Frenet(){};
  Frenet(const State &s, const State &d) : s(s), d(d){};
};

class Vehicle;

using FTrajectory = std::vector<Frenet>;
using LaneTraffic = std::vector<Vehicle>;
using Traffic = std::vector<LaneTraffic>;

struct Plan {
  Frenet target;
  FTrajectory trajectory;
  double t;
};

using TimePoint = std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

// Class to keep track of computation time
class Timer {
 public:
  static std::chrono::milliseconds ToMilliseconds(Duration duration) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration);
  }

  Timer() : elapsed_total(std::chrono::nanoseconds::zero()), iterations(0) {}
  ~Timer() {}

  TimePoint Start() { return Now(); }

  Duration Eval(TimePoint start) {
    auto end = Now();
    auto elapsed = end - start;
    elapsed_total += elapsed;
    ++iterations;
    return elapsed;
  }

  Duration AverageDuration() {
    if (iterations == 0) {
      return std::chrono::nanoseconds::zero();
    }
    return elapsed_total / iterations;
  }

  Duration TotalDuration() { return elapsed_total; }

  inline static TimePoint Now() { return std::chrono::steady_clock::now(); }

 private:
  Duration elapsed_total;
  size_t iterations;
};

/**
 * Converts from milliseconds to seconds
 */
inline double Ms2s(double x) { return x / 1000.0; }

/**
 * Convers from miles per hour to meters per second
 */
inline double Mph2ms(double x) { return x * 0.44704; }

/**
 * Computes the angle between the two components
 */
inline double Angle(double d_x, double d_y) {
  double angle = std::atan2(d_y, d_x);
  return angle < 0 ? (angle + 2 * M_PI) : angle;
}

/**
 * Converts degrees to radians
 */
inline double Deg2rad(double x) { return x * M_PI / 180; }

/**
 * Converts radians to degrees
 */
inline double Rad2deg(double x) { return x * 180 / M_PI; }

/**
 * Distance between two points
 */
inline double Distance(double x1, double y1, double x2, double y2) {
  double x_diff = x2 - x1;
  double y_diff = y2 - y1;
  return std::sqrt(x_diff * x_diff + y_diff * y_diff);
}

/**
 * Returns a value between 0 and 1 for positive x and a value between -1 and 0 for negative x
 */
inline double Logistic(double x) { return 2.0 / (1 + std::exp(-x)) - 1.0; }

}  // namespace PathPlanning

#endif
