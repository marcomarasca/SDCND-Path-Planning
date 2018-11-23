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
PathPlanning::LogConfig LOG_CONFIG;
}

std::string MAP_FILE_PATH = "../data/highway_map.csv";
double REFRESH_RATE = 100;
PathPlanning::TimePoint LAST_REFRESH = PathPlanning::Timer::Now();
bool DRAW_MODE = false;

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

  const double processing_time =
      PathPlanning::Ms2s(PathPlanning::Timer::ToMilliseconds(timer.AverageDuration()).count());

  planner.Update(telemetry, processing_time);

  if (DRAW_MODE) {
    PathPlanning::Duration delta = PathPlanning::Timer::Now() - LAST_REFRESH;

    if (PathPlanning::Timer::ToMilliseconds(delta).count() >= REFRESH_RATE) {
      planner.DrawRoad(processing_time);
      LAST_REFRESH = PathPlanning::Timer::Now();
    }
  }

  PathPlanning::Duration update_d = timer.Eval(update_s);

  PathPlanning::CTrajectory trajectory = planner.GetTrajectory();

  LOG(PathPlanning::INFO) << "Processing time: " << PathPlanning::Timer::ToMilliseconds(update_d).count()
                          << " ms (Average: " << PathPlanning::Timer::ToMilliseconds(timer.AverageDuration()).count()
                          << " ms)\n";

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
    if (DRAW_MODE) {
      PathPlanning::LOG_CONFIG.enabled = false;
      // Clear screen *unix only
      std::cout << __CLEAR_SCREEN__;
    }
  } else {
    LOG(PathPlanning::ERROR) << "Failed to listen to port";
    exit(EXIT_FAILURE);
  }

  h.run();
}

void PrintUsage() {
  std::cout << "Usage:\n\n   path_planning [options]\n" << std::endl;
  std::cout << "Options:" << std::endl;

  const size_t cols = 20;

  std::cout << std::left << std::setw(cols) << "   -f <filename>"
            << "= Custom path to the map file." << std::endl;
  std::cout << std::left << std::setw(cols) << "   -d,--debug"
            << "= Enable debug output." << std::endl;
  std::cout << std::left << std::setw(cols) << "   -g,--graphic"
            << "= Enable graphic mode (log disabled)." << std::endl;

  std::cout << std::endl;

  exit(EXIT_SUCCESS);
}

void ParseArguments(int argc, char *argv[]) {
  for (size_t i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      PrintUsage();
    } else if (arg == "-f" || arg == "--filename") {
      if (++i < argc) {
        MAP_FILE_PATH = argv[i];
      } else {
        std::cerr << "Path to the file name required.\n" << std::endl;
        PrintUsage();
      }
    } else if (arg == "-d" || arg == "--debug") {
      PathPlanning::LOG_CONFIG.level = PathPlanning::LogLevel::DEBUG;
    } else if (arg == "-g" || arg == "--graphic") {
      DRAW_MODE = true;
    } else {
      PrintUsage();
    }
  }
}

int main(int argc, char *argv[]) {
  ParseArguments(argc, argv);

  PathPlanning::Map map;

  try {
    map.LoadWaypoints(MAP_FILE_PATH);
  } catch (std::exception &e) {
    LOG(PathPlanning::ERROR) << e.what();
    exit(EXIT_FAILURE);
  }

  StartServer(map);
}
