#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <chrono>

// for portability of M_PI (Vis Studio, MinGW, etc.)
#ifndef M_PI
#define M_PI 3.14159265358979323846 /* pi */
#endif

namespace PathPlanning {

// Class to keep track of computation time
class Timer {
 public:
  typedef std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> TimePoint;
  typedef std::chrono::nanoseconds Duration;

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

 private:
  Duration elapsed_total;
  size_t iterations;

  inline TimePoint Now() { return std::chrono::steady_clock::now(); }
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
  return sqrt(x_diff * x_diff + y_diff * y_diff);
}

}  // namespace PathPlanning

#endif
