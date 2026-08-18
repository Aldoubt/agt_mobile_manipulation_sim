#include "agt_mission_core/mission_controller.hpp"

#include <utility>

namespace agt_mission_core
{

bool MissionController::start(std::vector<Waypoint> route)
{
  if (state_ != MissionState::IDLE || route.empty()) {
    return false;
  }

  route_ = std::move(route);
  current_waypoint_index_ = 0;
  state_ = MissionState::NAVIGATING;
  return true;
}

bool MissionController::onNavigationSucceeded()
{
  if (state_ != MissionState::NAVIGATING || route_.empty() ||
      current_waypoint_index_ >= route_.size()) {
    return false;
  }

  if (route_[current_waypoint_index_].behavior == WaypointBehavior::OPERATE) {
    state_ = MissionState::SETTLING;
    return true;
  }

  if (current_waypoint_index_ + 1 >= route_.size()) {
    state_ = MissionState::COMPLETED;
    return true;
  }

  ++current_waypoint_index_;
  state_ = MissionState::NAVIGATING;
  return true;
}

bool MissionController::onNavigationFailed()
{
  if (state_ != MissionState::NAVIGATING) {
    return false;
  }

  state_ = MissionState::FAILED;
  return true;
}

MissionState MissionController::state() const noexcept
{
  return state_;
}

std::size_t MissionController::currentWaypointIndex() const noexcept
{
  return current_waypoint_index_;
}

bool MissionController::armOperationPermitted() const noexcept
{
  return state_ == MissionState::ARM_READY;
}

}  // namespace agt_mission_core
