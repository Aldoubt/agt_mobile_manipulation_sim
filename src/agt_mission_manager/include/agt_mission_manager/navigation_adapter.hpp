#pragma once

#include "agt_mission_core/mission_state.hpp"

namespace agt_mission_manager
{

class INavigationAdapter
{
public:
  virtual ~INavigationAdapter() = default;

  virtual bool navigateTo(const agt_mission_core::Waypoint & waypoint) = 0;
  virtual bool cancel() = 0;
};

}  // namespace agt_mission_manager
