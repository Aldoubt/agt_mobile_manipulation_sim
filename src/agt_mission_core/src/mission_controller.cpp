#include "agt_mission_core/mission_controller.hpp"

#include <cmath>
#include <utility>

namespace agt_mission_core
{

MissionController::MissionController(SettlingConfig settling_config)
: settling_config_(settling_config)
{
}

bool MissionController::start(std::vector<Waypoint> route)
{
  if (state_ != MissionState::IDLE || route.empty()) {
    return false;
  }

  route_ = std::move(route);
  current_waypoint_index_ = 0;
  stable_time_ = 0.0;
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
    stable_time_ = 0.0;
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

bool MissionController::onBaseMotionSample(
  double linear_speed, double angular_speed, double dt)
{
  if (state_ != MissionState::SETTLING || dt <= 0.0) {
    return false;
  }

  const bool stopped =
    std::abs(linear_speed) <= settling_config_.max_linear_speed &&
    std::abs(angular_speed) <= settling_config_.max_angular_speed;

  if (!stopped) {
    stable_time_ = 0.0;
    return true;
  }

  stable_time_ += dt;
  if (stable_time_ + 1e-12 >= settling_config_.required_stable_time) {
    state_ = MissionState::ARM_READY;
  }
  return true;
}

bool MissionController::onArmMotionStarted()
{
  if (state_ != MissionState::ARM_READY) {
    return false;
  }
  state_ = MissionState::ARM_MOVING;
  return true;
}

bool MissionController::onArmMotionSucceeded()
{
  if (state_ != MissionState::ARM_MOVING) {
    return false;
  }
  state_ = MissionState::ARM_READY;
  return true;
}

bool MissionController::onArmMotionFailed()
{
  if (state_ != MissionState::ARM_MOVING) {
    return false;
  }
  state_ = MissionState::FAILED;
  return true;
}

bool MissionController::onRetractStarted()
{
  if (state_ != MissionState::ARM_READY) {
    return false;
  }
  state_ = MissionState::RETRACTING;
  return true;
}

bool MissionController::onRetractSucceeded()
{
  if (state_ != MissionState::RETRACTING || route_.empty()) {
    return false;
  }

  if (current_waypoint_index_ + 1 >= route_.size()) {
    state_ = MissionState::COMPLETED;
    return true;
  }

  ++current_waypoint_index_;
  state_ = MissionState::NAVIGATING;
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
