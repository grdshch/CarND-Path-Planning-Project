#ifndef PATH_PLANNING_STATES_H
#define PATH_PLANNING_STATES_H
#include <vector>
#include "helpers.h"

class Planner;

class State {
 public:
  State(Planner* planner) : planner_(planner) {}

  // do state logic and update planner parameters
  virtual void Update(InputData &data) = 0;

 protected:
  // check if part of current lane from s_min to s_max is free from other cars
  bool IsLaneFree(InputData &data, int lane, std::size_t steps, double& max_speed, int s_min, int s_max);

  // calculate distances of closest cars on each lane
  std::vector<double> ClosestCar(InputData &data);
  Planner* planner_;
};


// keep current lane until it's not possible to continue with max speed
class KeepLaneState : public State {
 public:
  KeepLaneState(Planner* planner) : State(planner) {}
  void Update(InputData &data) override;
};

// prepare for lane changing - find lane, wait for suitable moment
class FindLaneState : public State {
public:
  FindLaneState(Planner* planner) : State(planner) {}
  void Update(InputData &data) override;
};

// change the lane and wait to complete changing
class ChangeLaneState : public State {
 public:
  ChangeLaneState(Planner* planner, int lane);
  void Update(InputData &data) override;
 private:
  int lane_;
};


#endif //PATH_PLANNING_STATES_H
