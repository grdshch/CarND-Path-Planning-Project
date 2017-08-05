#ifndef PATH_PLANNING_STATES_H
#define PATH_PLANNING_STATES_H

class Planner;

class State {
 public:
  State() {}
  State(Planner* planner) : planner_(planner) {}
  virtual void Update() = 0;
 private:
  Planner* planner_;
};

class KeepLane : public State {
 public:
  void Update() {}
};

#endif //PATH_PLANNING_STATES_H
