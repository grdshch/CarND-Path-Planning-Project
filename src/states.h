#ifndef PATH_PLANNING_STATES_H
#define PATH_PLANNING_STATES_H
#include <vector>
#include "helpers.h"

class Planner;

class State {
 public:
  State(Planner* planner) : planner_(planner) {}
  //virtual std::pair<std::vector<double>, std::vector<double>> GetPlan(InputData& data) = 0;
  virtual std::pair<double, int> /* max speed, lane */ Sense(InputData &data, double max_speed, int lane) = 0;
 protected:
  Planner* planner_;
};

class KeepLane : public State {
 public:
  KeepLane(Planner* planner) : State(planner) {}
  //std::pair<std::vector<double>, std::vector<double>> GetPlan(InputData& data) override;
  std::pair<double, int> Sense(InputData &data, double max_speed, int lane) override;
 private:
  bool IsLaneOccupied(InputData &data, int lane, std::size_t steps, double& max_speed, int s_min, int s_max);
};

#endif //PATH_PLANNING_STATES_H
