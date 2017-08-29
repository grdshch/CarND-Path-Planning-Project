#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <vector>

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, std::vector<double> maps_x, std::vector<double> maps_y);

int NextWaypoint(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, std::vector<double> maps_s, std::vector<double> maps_x, std::vector<double> maps_y);

struct InputData {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  // Previous path data given to the Planner
  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;
  // Previous path's end s and d values
  double end_path_s;
  double end_path_d;

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  std::vector<std::vector<double>> sensor_fusion;

  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
};



#endif //PATH_PLANNING_HELPERS_H
