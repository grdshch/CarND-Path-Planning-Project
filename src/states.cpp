#include <math.h>
#include "spline.h"
#include "states.h"



bool KeepLane::IsLaneOccupied(InputData &data, int lane, std::size_t steps, double& max_speed, int s_min, int s_max) {
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
        return true;
      }
    }
  }
  return false;
}

std::pair<double, int> KeepLane::Sense(InputData &data, double max_speed, int lane) {
  std::size_t prev_size = data.previous_path_x.size();
  double speed_current, speed_other;
  if (IsLaneOccupied(data, lane, prev_size, speed_current, data.car_s, data.car_s + 20)) {
    bool changed = false;
    std::vector<int> other_lanes;
    if (lane == 1) other_lanes = {0, 2};
    else other_lanes = {1};
    for (auto& l: other_lanes) {
      if (!IsLaneOccupied(data, l, prev_size, speed_other, data.car_s - 3, data.car_s + 20)) {
        lane = l;
        changed = true;
        break;
      }
    }
    if (!changed) {
      return std::make_pair(std::min(max_speed, speed_current), lane);
    }
  }
  return std::make_pair(max_speed, lane);
}



