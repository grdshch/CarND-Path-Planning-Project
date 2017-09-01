#include <cmath>
#include "spline.h"
#include "planner.h"

static const double MH2MS = 1608. / 3600.;  // miles per hour to meters per second
static const double ACC = 0.2;  // max acceleration per call
static const double TARGET = 30.0; // distance to target to calculate
static const double DT = 0.02; // time duration
static const int POINTS = 50; // number of points to plan
static const int LANE_WIDTH = 4;

Planner::Planner() : lane_(1), state_(nullptr) {
  SetState(new KeepLaneState(this));
}

Planner::~Planner() {
  if (state_) {
    delete state_;
    state_ = nullptr;
  }
}

void Planner::SetState(State* state) {
  if (state_) delete state_;
  state_ = state;
}

void Planner::UpdateState(InputData& data) {
  state_->Update(data);
}

std::pair<std::vector<double>, std::vector<double>> Planner::GetPlan(InputData& data) {
  std::size_t prev_size = data.previous_path_x.size();
  if (prev_size > 0) data.car_s = data.end_path_s;

  max_speed_ = 49.6 * MH2MS; // < 50 miles per hour

  state_->Update(data);

  if (ref_v_ < max_speed_) {
    ref_v_ = std::min(ref_v_ + ACC, max_speed_);
  }
  else if (ref_v_ > max_speed_) {
    ref_v_ -= ACC;
  }
  return Predict(data);
}

std::pair<std::vector<double>, std::vector<double>> Planner::Predict(InputData &data) {
  std::size_t prev_size = data.previous_path_x.size();

  // x and y for points to calculate spline
  std::vector<double> X;
  std::vector<double> Y;

  double ref_x = data.car_x;
  double ref_y = data.car_y;
  double ref_yaw = data.car_yaw;

  if (prev_size < 2) {  // predict previous position of the car
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

  // use s + 30, s + 60 and s + 90 points
  for (std::size_t i = 1; i < 4; ++i) {
    std::vector<double> pt = getXY(data.car_s + i * TARGET, LANE_WIDTH * (0.5 + lane_),
                                   data.map_waypoints_s, data.map_waypoints_x, data.map_waypoints_y);
    X.push_back(pt[0]);
    Y.push_back(pt[1]);
  }

  // convert to car's local coordinate system
  for (std::size_t i = 0; i < X.size(); ++i) {
    double shift_x = X[i] - ref_x;
    double shift_y = Y[i] - ref_y;
    X[i] = (shift_x * std::cos(0 - ref_yaw) - shift_y * std::sin(0 - ref_yaw));
    Y[i] = (shift_x * std::sin(0 - ref_yaw) + shift_y * std::cos(0 - ref_yaw));
  }

  // calculate spline for local coordinates
  tk::spline spline;
  spline.set_points(X, Y);

  std::vector<double> next_x_vals;
  std::vector<double> next_y_vals;

  // use previously built path
  for (std::size_t i = 0; i < data.previous_path_x.size(); ++i) {
    next_x_vals.push_back(data.previous_path_x[i]);
    next_y_vals.push_back(data.previous_path_y[i]);
  }

  double target_x = TARGET;
  double target_y = spline(target_x);
  double target_dist = std::sqrt(target_x * target_x + target_y * target_y);

  double x_add_on = 0;
  double n = target_dist / (DT * ref_v_);
  // supplement previously built path with points from spline
  for (std::size_t i = 0; i < POINTS - data.previous_path_x.size(); ++i) {
    double x_point = target_x / n + x_add_on;

    double y_point = spline(x_point);
    x_add_on = x_point;

    // convert back to global map coordinates
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
