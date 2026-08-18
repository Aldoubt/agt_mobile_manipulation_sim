#pragma once

#include <cstddef>
#include <vector>

#include "agt_mission_core/mission_state.hpp"

namespace agt_mission_core
{

class MissionController
{
public:
  bool start(std::vector<Waypoint> route);
  bool onNavigationSucceeded();
  bool onNavigationFailed();

  MissionState state() const noexcept;
  std::size_t currentWaypointIndex() const noexcept;
  bool armOperationPermitted() const noexcept;

private:
  std::vector<Waypoint> route_;
  std::size_t current_waypoint_index_{0};
  MissionState state_{MissionState::IDLE};
};

}  // namespace agt_mission_core
