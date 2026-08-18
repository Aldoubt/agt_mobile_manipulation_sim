#include "agt_mission_manager/mission_runtime.hpp"

#include <utility>

namespace agt_mission_manager
{

MissionRuntime::MissionRuntime(
  INavigationAdapter & navigation,
  IManipulationAdapter & manipulation)
: navigation_(navigation), manipulation_(manipulation)
{
}

bool MissionRuntime::start(std::vector<agt_mission_core::Waypoint> route)
{
  if (route.empty() || !controller_.start(route)) {
    return false;
  }

  route_ = std::move(route);
  if (!navigation_.navigateTo(route_.front())) {
    controller_.onNavigationFailed();
    return false;
  }
  return true;
}

bool MissionRuntime::onNavigationSucceeded()
{
  if (!controller_.onNavigationSucceeded()) {
    return false;
  }

  if (controller_.state() == agt_mission_core::MissionState::NAVIGATING) {
    const auto index = controller_.currentWaypointIndex();
    if (index >= route_.size() || !navigation_.navigateTo(route_[index])) {
      controller_.onNavigationFailed();
      return false;
    }
  }
  return true;
}

bool MissionRuntime::onNavigationFailed()
{
  return controller_.onNavigationFailed();
}

bool MissionRuntime::onBaseMotionSample(
  double linear_speed, double angular_speed, double dt)
{
  return controller_.onBaseMotionSample(linear_speed, angular_speed, dt);
}

bool MissionRuntime::requestArmOperation(
  const std::string & target_id,
  const std::string & pose_id)
{
  if (!controller_.armOperationPermitted()) {
    return false;
  }
  if (!manipulation_.planToNamedTarget(target_id, pose_id)) {
    return false;
  }
  if (!controller_.onArmMotionStarted()) {
    return false;
  }
  if (!manipulation_.executeLastPlan()) {
    controller_.onArmMotionFailed();
    return false;
  }
  return true;
}

bool MissionRuntime::onArmMotionSucceeded()
{
  return controller_.onArmMotionSucceeded();
}

bool MissionRuntime::onArmMotionFailed()
{
  return controller_.onArmMotionFailed();
}

bool MissionRuntime::requestRetract()
{
  if (!controller_.armOperationPermitted()) {
    return false;
  }
  if (!manipulation_.retract()) {
    return false;
  }
  return controller_.onRetractStarted();
}

bool MissionRuntime::onRetractSucceeded()
{
  const bool accepted = controller_.onRetractSucceeded();
  if (!accepted) {
    return false;
  }

  if (controller_.state() == agt_mission_core::MissionState::NAVIGATING) {
    const auto index = controller_.currentWaypointIndex();
    if (index >= route_.size() || !navigation_.navigateTo(route_[index])) {
      controller_.onNavigationFailed();
      return false;
    }
  }
  return true;
}

agt_mission_core::MissionState MissionRuntime::state() const noexcept
{
  return controller_.state();
}

}  // namespace agt_mission_manager
