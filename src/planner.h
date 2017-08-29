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
  State* state_;
};

#endif //PATH_PLANNING_PLANNER_H
