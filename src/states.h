#ifndef PATH_PLANNING_STATES_H
#define PATH_PLANNING_STATES_H

#include <vector>
#include "helpers.h"

class Planner;

class State {
 public:
  State() {}
  State(Planner* planner) : planner_(planner) {}
  virtual std::pair<std::vector<double>, std::vector<double>> GetPlan(InputData& data) = 0;
 private:
  Planner* planner_;
};

class KeepLane : public State {
 public:
  KeepLane() : lane_(1), ref_v_(0) {};
  std::pair<std::vector<double>, std::vector<double>> GetPlan(InputData& data) override;
 private:
  int lane_;
  double ref_v_;

};

#endif //PATH_PLANNING_STATES_H
