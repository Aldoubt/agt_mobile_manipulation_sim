#pragma once

#include <string>

namespace agt_mission_core
{

enum class MissionState
{
  IDLE,
  NAVIGATING,
  SETTLING,
  ARM_READY,
  ARM_MOVING,
  RETRACTING,
  COMPLETED,
  FAILED,
};

enum class WaypointBehavior
{
  PASS,
  OPERATE,
};

struct Waypoint
{
  std::string id;
  WaypointBehavior behavior{WaypointBehavior::PASS};
};

}  // namespace agt_mission_core
