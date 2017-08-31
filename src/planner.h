#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>

#include "states.h"
#include "helpers.h"

class Planner {
public:
  Planner();
  ~Planner();
  std::pair<std::vector<double>, std::vector<double>> GetPlan(InputData& data);
private:
  std::pair<std::vector<double>, std::vector<double>> Predict(InputData& data);
  State* state_;
  int lane_;
  double ref_v_;
};

#endif //PATH_PLANNING_PLANNER_H
