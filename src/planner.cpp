#include "planner.h"

Planner::Planner() {
  state_ = new KeepLane();
}

Planner::~Planner() {
  delete state_;
}

std::pair<std::vector<double>, std::vector<double>> Planner::GetPlan(InputData& data) {
  return state_->GetPlan(data);
};
