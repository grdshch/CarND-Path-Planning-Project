#include <math.h>
#include "spline.h"
#include "states.h"

std::pair<std::vector<double>, std::vector<double>> KeepLane::GetPlan(InputData& data) {
  size_t prev_size = data.previous_path_x.size();

  if (prev_size > 0) data.car_s = data.end_path_s;

  double max_speed = 49.5;
  bool hindrance = false;

  std::vector<double> closest_car = {1000., 1000., 1000.};
  for (auto& sensor_data: data.sensor_fusion) {
    double d = sensor_data[6];
    int lane = d / 4;
    if (sensor_data[5] > data.car_s) {
      closest_car[lane] = std::min(closest_car[lane], sensor_data[5]);
    }
  }

  for (auto& sensor_data: data.sensor_fusion) {
    double d = sensor_data[6];
    if (d > 4 * lane_ && d < 4 * (lane_ + 1)) {
      double vx = sensor_data[3];
      double vy = sensor_data[4];
      double speed = std::sqrt(vx * vx + vy * vy);
      double s = sensor_data[5];
      s += prev_size * 0.02 * speed;
      if (s > data.car_s && s < data.car_s + 30) {
        std::vector<double> other_lanes;
        if (lane_ == 1) other_lanes = {0, 2};
        else other_lanes = {1};
        for (auto l: other_lanes) {
          if ()
        }
        //max_speed = speed;
        hindrance = true;
        if (lane_ > 0) lane_--;
        else lane_++;
        break;
      }

    }
  }

  if (hindrance || ref_v_ > max_speed) {
    //ref_v -= 0.5;
  }
  else if (ref_v_ < max_speed) {
    ref_v_ += 0.5;
  }


  std::vector<double> X;
  std::vector<double> Y;

  double ref_x = data.car_x;
  double ref_y = data.car_y;
  double ref_yaw = data.car_yaw;

  if (prev_size < 2) {
    X.push_back(data.car_x - std::cos(data.car_yaw));
    X.push_back(data.car_x);

    Y.push_back(data.car_y - std::sin(data.car_yaw));
    Y.push_back(data.car_y);
  }
  else {
    ref_x = data.previous_path_x.end()[-1];
    ref_y = data.previous_path_y.end()[-1];
    double prev_ref_x = data.previous_path_x.end()[-2];
    double prev_ref_y = data.previous_path_y.end()[-2];
    ref_yaw = std::atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);

    X.push_back(prev_ref_x);
    X.push_back(ref_x);

    Y.push_back(prev_ref_y);
    Y.push_back(ref_y);
  }

  for (size_t i = 1; i < 4; ++i) {
    std::vector<double> pt = getXY(data.car_s + i * 30, 2 + 4 * lane_,
                                   data.map_waypoints_s, data.map_waypoints_x, data.map_waypoints_y);
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

  for (size_t i = 0; i < data.previous_path_x.size(); ++i) {
    next_x_vals.push_back(data.previous_path_x[i]);
    next_y_vals.push_back(data.previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = std::sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;

  for (size_t i = 0; i < 50 - data.previous_path_x.size(); ++i) {
    double n = target_dist / (0.02 * ref_v_ / 2.24);
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
  return std::make_pair(next_x_vals, next_y_vals);

}
