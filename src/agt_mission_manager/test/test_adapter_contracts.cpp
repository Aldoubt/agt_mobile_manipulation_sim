#include <cassert>
#include <string>
#include <vector>

#include "agt_mission_manager/manipulation_adapter.hpp"
#include "agt_mission_manager/mission_runtime.hpp"
#include "agt_mission_manager/navigation_adapter.hpp"

using agt_mission_core::MissionState;
using agt_mission_core::Waypoint;
using agt_mission_core::WaypointBehavior;
using agt_mission_manager::IManipulationAdapter;
using agt_mission_manager::INavigationAdapter;
using agt_mission_manager::MissionRuntime;

namespace
{

class FakeNavigationAdapter final : public INavigationAdapter
{
public:
  bool navigateTo(const Waypoint & waypoint) override
  {
    ++navigate_calls;
    last_waypoint = waypoint.id;
    return true;
  }

  bool cancel() override
  {
    ++cancel_calls;
    return true;
  }

  int navigate_calls{0};
  int cancel_calls{0};
  std::string last_waypoint;
};

class FakeManipulationAdapter final : public IManipulationAdapter
{
public:
  bool planToNamedTarget(const std::string & target_id, const std::string & pose_id) override
  {
    ++plan_calls;
    last_target = target_id;
    last_pose = pose_id;
    return true;
  }

  bool executeLastPlan() override
  {
    ++execute_calls;
    return true;
  }

  bool retract() override
  {
    ++retract_calls;
    return true;
  }

  int plan_calls{0};
  int execute_calls{0};
  int retract_calls{0};
  std::string last_target;
  std::string last_pose;
};

}  // namespace

int main()
{
  FakeNavigationAdapter navigation;
  FakeManipulationAdapter manipulation;
  MissionRuntime runtime(navigation, manipulation);

  const std::vector<Waypoint> route{{"P2", WaypointBehavior::OPERATE}};
  assert(runtime.start(route));
  assert(navigation.navigate_calls == 1);
  assert(navigation.last_waypoint == "P2");
  assert(runtime.state() == MissionState::NAVIGATING);

  assert(!runtime.requestArmOperation("tomato_001", "front"));
  assert(manipulation.plan_calls == 0);
  assert(manipulation.execute_calls == 0);

  assert(runtime.onNavigationSucceeded());
  assert(runtime.state() == MissionState::SETTLING);
  assert(!runtime.requestArmOperation("tomato_001", "front"));
  assert(manipulation.plan_calls == 0);

  assert(runtime.onBaseMotionSample(0.0, 0.0, 0.50));
  assert(runtime.state() == MissionState::ARM_READY);
  assert(runtime.requestArmOperation("tomato_001", "front"));
  assert(runtime.state() == MissionState::ARM_MOVING);
  assert(manipulation.plan_calls == 1);
  assert(manipulation.execute_calls == 1);
  assert(manipulation.last_target == "tomato_001");
  assert(manipulation.last_pose == "front");

  assert(runtime.onArmMotionSucceeded());
  assert(runtime.state() == MissionState::ARM_READY);
  assert(runtime.requestRetract());
  assert(runtime.state() == MissionState::RETRACTING);
  assert(manipulation.retract_calls == 1);
  assert(runtime.onRetractSucceeded());
  assert(runtime.state() == MissionState::COMPLETED);

  return 0;
}
