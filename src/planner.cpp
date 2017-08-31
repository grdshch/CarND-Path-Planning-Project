#include <cmath>
#include "spline.h"
#include "planner.h"

static const double MH2MS = 1608. / 3600.;  // miles per hour to meters per second

Planner::Planner() : lane_(1){
  state_ = new KeepLane(this);
}

Planner::~Planner() {
  delete state_;
}

std::pair<std::vector<double>, std::vector<double>> Planner::GetPlan(InputData& data) {
  std::size_t prev_size = data.previous_path_x.size();
  if (prev_size > 0) data.car_s = data.end_path_s;

  double max_speed = 49.6 * MH2MS;

  auto p = state_->Sense(data, max_speed, lane_);
  max_speed = p.first;
  lane_ = p.second;

  if (ref_v_ < max_speed) {
    ref_v_ = std::min(ref_v_ + 0.5, max_speed);
  }
  else if (ref_v_ > max_speed) {
    ref_v_ -= 0.5;
  }
  return Predict(data);
}

std::pair<std::vector<double>, std::vector<double>> Planner::Predict(InputData &data) {
  std::size_t prev_size = data.previous_path_x.size();

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
  } else {
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

  for (std::size_t i = 1; i < 4; ++i) {
    std::vector<double> pt = getXY(data.car_s + i * 30, 2 + 4 * lane_,
                                   data.map_waypoints_s, data.map_waypoints_x, data.map_waypoints_y);
    X.push_back(pt[0]);
    Y.push_back(pt[1]);
  }

  for (std::size_t i = 0; i < X.size(); ++i) {
    double shift_x = X[i] - ref_x;
    double shift_y = Y[i] - ref_y;
    X[i] = (shift_x * std::cos(0 - ref_yaw) - shift_y * std::sin(0 - ref_yaw));
    Y[i] = (shift_x * std::sin(0 - ref_yaw) + shift_y * std::cos(0 - ref_yaw));
  }

  tk::spline spline;
  spline.set_points(X, Y);

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  for (std::size_t i = 0; i < data.previous_path_x.size(); ++i) {
    next_x_vals.push_back(data.previous_path_x[i]);
    next_y_vals.push_back(data.previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = spline(target_x);
  double target_dist = std::sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;
  double n = target_dist / (0.02 * ref_v_);
  for (std::size_t i = 0; i < 50 - data.previous_path_x.size(); ++i) {
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
