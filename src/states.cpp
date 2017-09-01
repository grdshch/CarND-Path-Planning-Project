#include <cmath>
#include "spline.h"
#include "states.h"
#include "planner.h"

bool State::IsLaneFree(InputData &data, int lane, std::size_t steps, double& max_speed, int s_min, int s_max) {
  for (std::size_t i = 0; i < data.sensor_fusion.size(); ++i) {
    auto sensor_data = data.sensor_fusion[i];
    double d = sensor_data[6];
    if (4 * lane < d && d < 4 * (lane + 1)) {
      double vx = sensor_data[3];
      double vy = sensor_data[4];
      double speed = std::sqrt(vx * vx + vy * vy);
      double s = sensor_data[5];
      s += steps * 0.02 * speed;
      if (s_min < s && s < s_max) {
        max_speed = speed;
        return false;
      }
    }
  }
  return true;
}

std::vector<double> State::ClosestCar(InputData &data) {
  std::vector<double> dist = {1000, 1000, 1000};
  for (std::size_t i = 0; i < data.sensor_fusion.size(); ++i) {
    auto sensor_data = data.sensor_fusion[i];
    double s = sensor_data[5];
    if (s >= data.car_s - 3) {
      double d = sensor_data[6];
      int lane = std::floor(d / 4);
      dist[lane] = std::min(dist[lane], s - data.car_s);
    }
  }
  return dist;
}

void KeepLaneState::Update(InputData &data) {
  std::size_t prev_size = data.previous_path_x.size();
  double speed;
  int lane = planner_->GetLane();

  if (!IsLaneFree(data, lane, prev_size, speed, data.car_s, data.car_s + 30)) {
    planner_->SetState(new FindLaneState(planner_));
  }
}

void FindLaneState::Update(InputData &data) {
  std::size_t prev_size = data.previous_path_x.size();
  int lane = planner_->GetLane();
  double max_speed = planner_->GetMaxSpeed();

  // selecting new lane
  int new_lane;
  if (lane == 1) {
    // from lanes 0 and 2 select one which has longer distance to the nearest car
    auto closest_car = ClosestCar(data);
    new_lane = closest_car[0] > closest_car[2] ? 0 : 2;
  }
  else {
    // there is only one choice for lanes 0 and 2
    new_lane = 1;
  }

  double speed = max_speed;
  if (IsLaneFree(data, new_lane, prev_size, speed, data.car_s - 3, data.car_s + 20)) {
    planner_->SetState(new ChangeLaneState(planner_, new_lane));
  }
  else {
    if (!IsLaneFree(data, lane, prev_size, speed, data.car_s, data.car_s + 25)) {
      planner_->SetMaxSpeed(std::min(max_speed, speed));
    }
  }
}

ChangeLaneState::ChangeLaneState(Planner* planner, int lane) : State(planner), lane_(lane) {
  planner_->SetLane(lane);
}

void ChangeLaneState::Update(InputData &data) {
  if (std::fabs(data.car_d - (4 * lane_ + 2)) < 1) {
    planner_->SetState(new KeepLaneState(planner_));
  }
}