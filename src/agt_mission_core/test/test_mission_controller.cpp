#include <cassert>
#include <vector>

#include "agt_mission_core/mission_controller.hpp"

using agt_mission_core::MissionController;
using agt_mission_core::MissionState;
using agt_mission_core::Waypoint;
using agt_mission_core::WaypointBehavior;

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

  return 0;
}
