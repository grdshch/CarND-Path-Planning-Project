#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "helpers.h"

#include "spline.h"

// for convenience
using json = nlohmann::json;

static const double MH2MS = 1608. / 3600.;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in std::string format will be returned,
// else the empty std::string "" will be returned.
std::string hasData(std::string s) {
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

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Waypoint map to read from
  std::string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  double vd = 0;
  double v = 0;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&vd, &v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        std::string event = j[0].get<std::string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          std::vector<double> X;
          std::vector<double> Y;

          double d = car_d;
          double s = car_s;
          //double v = car_speed * MH2MS;  // speed in meters per second
          size_t next_waypoint;

          for (size_t i = 0; i < map_waypoints_s.size(); ++i) {
            if (map_waypoints_s[i] > car_s) {
              next_waypoint = i;
              break;
            }
          }

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;
          double x = car_x;
          double y = car_y;

          double dt = 0.02;

          X.clear(); Y.clear();
          std::vector<double> S;
          for (size_t i = 0; i < 4; ++i) {
            size_t ind = next_waypoint - 2 + i;
            X.push_back(map_waypoints_x[ind] + map_waypoints_dx[ind] * 6);
            Y.push_back(map_waypoints_y[ind] + map_waypoints_dy[ind] * 6);
            S.push_back(map_waypoints_s[ind]);
          }
          tk::spline xs_spline;
          tk::spline ys_spline;
          xs_spline.set_points(S, X);
          ys_spline.set_points(S, Y);
          next_x_vals.clear();
          next_y_vals.clear();
          double new_s = s;
          for (size_t i = 0; i < 50; ++i) {
            new_s += v * dt;
            if (vd < 5 && v < 40 * MH2MS) {
              vd += 1;
              v += vd / 50.;
            }
            else if (vd > 0){
              vd -= 1;
              v += vd / 50.;
            }

            std::cout << vd << "  " << v / MH2MS << std::endl;
            double new_x = xs_spline(new_s);
            double new_y = ys_spline(new_s);
            next_x_vals.push_back(new_x);
            next_y_vals.push_back(new_y);
          }

          //next_x_vals = {car_x + 1, car_x + 2, car_x + 3};
          //next_y_vals = {car_y, car_y, car_y};

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































