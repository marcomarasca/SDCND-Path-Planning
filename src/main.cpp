#include <uWS/uWS.h>
#include <string>
#include <vector>

#include "json.hpp"
#include "map.h"
#include "utils.h"

// for convenience
using json = nlohmann::json;

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

void ProcessTelemetry(uWS::WebSocket<uWS::SERVER> &ws, PathPlanning::Map &map, PathPlanning::Timer &timer,
                      json &telemetry) {
  double car_x = telemetry[1]["x"];
  double car_y = telemetry[1]["y"];
  double car_s = telemetry[1]["s"];
  double car_d = telemetry[1]["d"];
  double car_yaw = telemetry[1]["yaw"];
  double car_speed = telemetry[1]["speed"];

  // Previous path data given to the Planner
  auto previous_path_x = telemetry[1]["previous_path_x"];
  auto previous_path_y = telemetry[1]["previous_path_y"];
  // Previous path's end s and d values
  double end_path_s = telemetry[1]["end_path_s"];
  double end_path_d = telemetry[1]["end_path_d"];

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  auto sensor_fusion = telemetry[1]["sensor_fusion"];

  json msgJson;

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
  msgJson["next_x"] = next_x_vals;
  msgJson["next_y"] = next_y_vals;

  auto msg = "42[\"control\"," + msgJson.dump() + "]";

  SendMessage(ws, msg);
}

void StartServer(PathPlanning::Map &map) {
  uWS::Hub h;
  PathPlanning::Timer timer;

  h.onMessage([&map, &timer](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          ProcessTelemetry(ws, map, timer, json_obj[1]);
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
      [](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) { std::cout << "Connected!!!" << std::endl; });

  h.onDisconnection([](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    exit(EXIT_FAILURE);
  }

  h.run();
}

int main(int argc, char *argv[]) {
  std::string map_file_path = "../data/highway_map.csv";

  if (argc > 1) {
    map_file_path = argv[1];
  }

  PathPlanning::Map map;

  try {
    map.LoadWaypoints(map_file_path);
  } catch (std::exception &e) {
    std::cout << e.what() << std::endl;
    exit(EXIT_FAILURE);
  }

  StartServer(map);
}
