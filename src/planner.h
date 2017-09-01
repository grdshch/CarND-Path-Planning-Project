#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>

#include "states.h"
#include "helpers.h"

class Planner {
public:
  Planner();
  ~Planner();

  void SetState(State* state);
  void UpdateState(InputData& data);

  std::pair<std::vector<double>, std::vector<double>> GetPlan(InputData& data);

  void SetMaxSpeed(double speed) {max_speed_ = speed;}
  double GetMaxSpeed() {return max_speed_;}

  void SetLane(int lane) {lane_ = lane;}
  int GetLane() {return lane_;}

private:
  std::pair<std::vector<double>, std::vector<double>> Predict(InputData& data);
  State* state_;  // current state
  int lane_;  // current lane
  double ref_v_;  // reference velocity
  double max_speed_;  // current max speed
};

#endif //PATH_PLANNING_PLANNER_H
