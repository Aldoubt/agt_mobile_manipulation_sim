#include <cassert>
#include <vector>

#include "agt_mission_core/mission_controller.hpp"

using agt_mission_core::MissionController;
using agt_mission_core::MissionState;
using agt_mission_core::Waypoint;
using agt_mission_core::WaypointBehavior;

namespace
{

MissionController makeSettlingController()
{
  MissionController controller;
  const std::vector<Waypoint> route{{"P2", WaypointBehavior::OPERATE}};
  assert(controller.start(route));
  assert(controller.onNavigationSucceeded());
  assert(controller.state() == MissionState::SETTLING);
  return controller;
}

}  // namespace

int main()
{
  {
    MissionController controller;
    const std::vector<Waypoint> route{{"P2", WaypointBehavior::OPERATE}};

    assert(controller.state() == MissionState::IDLE);
    assert(controller.start(route));
    assert(controller.state() == MissionState::NAVIGATING);
    assert(controller.onNavigationSucceeded());
    assert(controller.state() == MissionState::SETTLING);
    assert(!controller.armOperationPermitted());
  }

  {
    MissionController controller;
    const std::vector<Waypoint> route{{"P1", WaypointBehavior::PASS}};

    assert(controller.start(route));
    assert(controller.onNavigationFailed());
    assert(controller.state() == MissionState::FAILED);
  }

  {
    auto controller = makeSettlingController();
    assert(controller.onBaseMotionSample(0.0, 0.0, 0.49));
    assert(controller.state() == MissionState::SETTLING);
    assert(!controller.armOperationPermitted());

    assert(controller.onBaseMotionSample(0.0, 0.0, 0.01));
    assert(controller.state() == MissionState::ARM_READY);
    assert(controller.armOperationPermitted());
  }

  {
    auto controller = makeSettlingController();
    assert(controller.onBaseMotionSample(0.0, 0.0, 0.30));
    assert(controller.onBaseMotionSample(0.02, 0.0, 0.01));
    assert(controller.state() == MissionState::SETTLING);

    assert(controller.onBaseMotionSample(0.0, 0.0, 0.30));
    assert(controller.state() == MissionState::SETTLING);
    assert(controller.onBaseMotionSample(0.0, 0.0, 0.20));
    assert(controller.state() == MissionState::ARM_READY);
  }

  {
    MissionController controller;
    const std::vector<Waypoint> route{{"P2", WaypointBehavior::OPERATE}};
    assert(controller.start(route));
    assert(!controller.onArmMotionStarted());
    assert(controller.state() == MissionState::NAVIGATING);
  }

  {
    auto controller = makeSettlingController();
    assert(controller.onBaseMotionSample(0.0, 0.0, 0.50));
    assert(controller.onArmMotionStarted());
    assert(controller.state() == MissionState::ARM_MOVING);
    assert(!controller.armOperationPermitted());
    assert(controller.onArmMotionSucceeded());
    assert(controller.state() == MissionState::ARM_READY);
    assert(controller.armOperationPermitted());
  }

  {
    MissionController controller;
    const std::vector<Waypoint> route{
      {"P2", WaypointBehavior::OPERATE},
      {"P3", WaypointBehavior::PASS},
    };
    assert(controller.start(route));
    assert(controller.onNavigationSucceeded());
    assert(controller.onBaseMotionSample(0.0, 0.0, 0.50));
    assert(controller.onRetractStarted());
    assert(controller.state() == MissionState::RETRACTING);
    assert(controller.onRetractSucceeded());
    assert(controller.state() == MissionState::NAVIGATING);
    assert(controller.currentWaypointIndex() == 1U);
  }

  {
    auto controller = makeSettlingController();
    assert(controller.onBaseMotionSample(0.0, 0.0, 0.50));
    assert(controller.onRetractStarted());
    assert(controller.onRetractSucceeded());
    assert(controller.state() == MissionState::COMPLETED);
  }

  return 0;
}
