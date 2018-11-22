#include <uWS/uWS.h>
#include <string>
#include <vector>

#include "json.hpp"
#include "logger.h"
#include "map.h"
#include "path_planner.h"
#include "utils.h"

// for convenience
using json = nlohmann::json;
using PathPlanning::LOG;

namespace PathPlanning {
LogConfig LOG_CONFIG;
}

bool draw_mode = false;
PathPlanning::TimePoint last_update = PathPlanning::Timer::Now();

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string HasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

void SendMessage(uWS::WebSocket<uWS::SERVER> &ws, std::string msg) {
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

void ProcessTelemetry(uWS::WebSocket<uWS::SERVER> &ws, PathPlanning::PathPlanner &planner, PathPlanning::Timer &timer,
                      json &telemetry) {
  // Updates the state of the planner using the data from the telemetry
  PathPlanning::TimePoint update_s = timer.Start();

  double processing_time = PathPlanning::Ms2s(PathPlanning::Timer::ToMilliseconds(timer.AverageDuration()).count());

  processing_time = std::max(processing_time, PathPlanning::MIN_PROCESSING_TIME);

  planner.Update(telemetry, processing_time);

  if (draw_mode) {
    PathPlanning::Duration delta = PathPlanning::Timer::Now() - last_update;

    if (PathPlanning::Timer::ToMilliseconds(delta).count() >= 100) {
      planner.DrawRoad();
      last_update = PathPlanning::Timer::Now();
    }
  }

  PathPlanning::Duration update_d = timer.Eval(update_s);

  PathPlanning::CTrajectory trajectory = planner.GetTrajectory();

  LOG(PathPlanning::INFO) << "Processing time: " << PathPlanning::Timer::ToMilliseconds(update_d).count()
                          << " ms (Average: " << PathPlanning::Timer::ToMilliseconds(timer.AverageDuration()).count()
                          << " ms)";

  json msgJson;

  msgJson["next_x"] = trajectory.first;
  msgJson["next_y"] = trajectory.second;

  auto msg = "42[\"control\"," + msgJson.dump() + "]";

  SendMessage(ws, msg);
}

void StartServer(PathPlanning::Map &map) {
  uWS::Hub h;
  PathPlanning::Timer timer;
  PathPlanning::PathPlanner planner{map, PathPlanning::LANES_N};

  h.onMessage([&map, &planner, &timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    // std::string sdata = std::string(data).substr(0, length);
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      std::string message = HasData(data);
      if (message != "") {
        auto json_obj = json::parse(message);
        std::string event = json_obj[0].get<std::string>();
        if (event == "telemetry") {
          ProcessTelemetry(ws, planner, timer, json_obj[1]);
        }
      } else {
        // Manual driving
        SendMessage(ws, "42[\"manual\",{}]");
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection(
      [](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { LOG(PathPlanning::INFO) << "Connected!!!"; });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    LOG(PathPlanning::INFO) << "Disconnected";
  });

  int port = 4567;
  if (h.listen(port)) {
    LOG(PathPlanning::INFO) << "Listening to port " << port;

    // Configure draw mode
    if (draw_mode) {
      PathPlanning::LOG_CONFIG.enabled = false;
    }
  } else {
    LOG(PathPlanning::ERROR) << "Failed to listen to port";
    exit(EXIT_FAILURE);
  }

  h.run();
}

int main(int argc, char *argv[]) {
  std::string map_file_path = "../data/highway_map.csv";

  if (argc > 1) {
    map_file_path = argv[1];
  }

  // Configure logger/ draw mode
  draw_mode = false;
  PathPlanning::LOG_CONFIG.level = PathPlanning::LogLevel::DEBUG;

  PathPlanning::Map map;

  try {
    map.LoadWaypoints(map_file_path);
  } catch (std::exception &e) {
    LOG(PathPlanning::ERROR) << e.what();
    exit(EXIT_FAILURE);
  }

  StartServer(map);
}
