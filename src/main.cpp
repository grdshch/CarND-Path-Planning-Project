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

  size_t lane = 1;
  double ref_v = 0;

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_v](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          size_t prev_size = previous_path_x.size();

          if (prev_size > 0) car_s = end_path_s;

          double max_speed = 49.5;
          bool hindrance = false;

          for (auto& data: sensor_fusion) {
            double d = data[6];
            if (d > 4 * lane && d < 4 * (lane + 1)) {
              double vx = data[3];
              double vy = data[4];
              double speed = sqrt(vx * vx + vy * vy);
              double s = data[5];
              s += prev_size * 0.02 * speed;
              if (s > car_s && s < car_s + 30) {
                //max_speed = speed;
                hindrance = true;
                if (lane > 0) lane--;
                else lane++;
              }

            }
          }

          if (hindrance || ref_v > max_speed) {
            //ref_v -= 0.5;
          }
          else if (ref_v < max_speed) {
            ref_v += 0.5;
          }


          std::vector<double> X;
          std::vector<double> Y;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = car_yaw;

          if (previous_path_x.size() < 2) {
            X.push_back(car_x - cos(car_yaw));
            X.push_back(car_x);

            Y.push_back(car_y - sin(car_yaw));
            Y.push_back(car_y);
          }
          else {
            ref_x = previous_path_x.end()[-1];
            ref_y = previous_path_y.end()[-1];
            double prev_ref_x = previous_path_x.end()[-2];
            double prev_ref_y = previous_path_y.end()[-2];
            ref_yaw = std::atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

            X.push_back(prev_ref_x);
            X.push_back(ref_x);

            Y.push_back(prev_ref_y);
            Y.push_back(ref_y);
          }

          for (size_t i = 1; i < 4; ++i) {
            std::vector<double> pt = getXY(car_s + i * 30, 2 + 4 * lane,
                                           map_waypoints_s, map_waypoints_x, map_waypoints_y);
            X.push_back(pt[0]);
            Y.push_back(pt[1]);
          }

          for (size_t i = 0; i < X.size(); ++i) {
            double shift_x = X[i] - ref_x;
            double shift_y = Y[i] - ref_y;
            X[i] = (shift_x * std::cos(0 - ref_yaw) - shift_y * std::sin(0 - ref_yaw));
            Y[i] = (shift_x * std::sin(0 - ref_yaw) + shift_y * std::cos(0 - ref_yaw));
          }

          tk::spline spline;
          spline.set_points(X, Y);

          std::vector<double> next_x_vals;
          std::vector<double> next_y_vals;

          for (size_t i = 0; i < previous_path_x.size(); ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = spline(target_x);
          double target_dist = std::sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (size_t i = 0; i < 50 - previous_path_x.size(); ++i) {
            double n = target_dist / (0.02 * ref_v / 2.24);
            double x_point = target_x / n + x_add_on;
            double y_point = spline(x_point);
            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;
            x_point = x_ref * std::cos(ref_yaw) - y_ref * std::sin(ref_yaw);
            y_point = x_ref * std::sin(ref_yaw) + y_ref * std::cos(ref_yaw);

            x_point += ref_x;
            y_point += ref_y;
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          json msgJson;
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
