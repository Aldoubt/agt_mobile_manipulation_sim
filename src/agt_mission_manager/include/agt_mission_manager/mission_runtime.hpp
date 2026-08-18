#pragma once

#include <string>
#include <vector>

#include "agt_mission_core/mission_controller.hpp"
#include "agt_mission_manager/manipulation_adapter.hpp"
#include "agt_mission_manager/navigation_adapter.hpp"

namespace agt_mission_manager
{

class MissionRuntime
{
public:
  MissionRuntime(
    INavigationAdapter & navigation,
    IManipulationAdapter & manipulation);

  bool start(std::vector<agt_mission_core::Waypoint> route);
  bool onNavigationSucceeded();
  bool onNavigationFailed();
  bool onBaseMotionSample(double linear_speed, double angular_speed, double dt);
  bool requestArmOperation(const std::string & target_id, const std::string & pose_id);
  bool onArmMotionSucceeded();
  bool onArmMotionFailed();
  bool requestRetract();
  bool onRetractSucceeded();

  agt_mission_core::MissionState state() const noexcept;

private:
  INavigationAdapter & navigation_;
  IManipulationAdapter & manipulation_;
  agt_mission_core::MissionController controller_;
  std::vector<agt_mission_core::Waypoint> route_;
};

}  // namespace agt_mission_manager
