#pragma once

#include <string>

namespace agt_mission_manager
{

class IManipulationAdapter
{
public:
  virtual ~IManipulationAdapter() = default;

  virtual bool planToNamedTarget(
    const std::string & target_id,
    const std::string & pose_id) = 0;
  virtual bool executeLastPlan() = 0;
  virtual bool retract() = 0;
};

}  // namespace agt_mission_manager
