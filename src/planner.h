#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>

#include "states.h"

class Planner {
public:
  Planner() {}
  std::pair<double, double> GetPlan() { return std::make_pair(0., 0.);};
private:
  State* state;
};

#endif //PATH_PLANNING_PLANNER_H
