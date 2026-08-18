#pragma once

#include <cstddef>
#include <vector>

#include "agt_mission_core/mission_state.hpp"

namespace agt_mission_core
{

struct SettlingConfig
{
  double max_linear_speed{0.01};
  double max_angular_speed{0.01};
  double required_stable_time{0.5};
};

class MissionController
{
public:
  MissionController() = default;
  explicit MissionController(SettlingConfig settling_config);

  bool start(std::vector<Waypoint> route);
  bool onNavigationSucceeded();
  bool onNavigationFailed();
  bool onBaseMotionSample(double linear_speed, double angular_speed, double dt);
  bool onArmMotionStarted();
  bool onArmMotionSucceeded();
  bool onArmMotionFailed();
  bool onRetractStarted();
  bool onRetractSucceeded();

  MissionState state() const noexcept;
  std::size_t currentWaypointIndex() const noexcept;
  bool armOperationPermitted() const noexcept;

private:
  std::vector<Waypoint> route_;
  std::size_t current_waypoint_index_{0};
  MissionState state_{MissionState::IDLE};
  SettlingConfig settling_config_{};
  double stable_time_{0.0};
};

}  // namespace agt_mission_core
