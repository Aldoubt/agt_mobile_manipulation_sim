# AGT Mobile Manipulation Sim

A reusable ROS 2 simulation framework for navigation-manipulation research with configurable mobile bases, robotic arms, greenhouse environments, semantic targets, mission execution, and interactive motion-planning experiments.

## V0.1 scope

The first baseline targets a known-world mobile-manipulation workflow:

1. follow a predefined route or waypoint sequence;
2. stop at an operation waypoint and verify the base has settled;
3. select a known tomato target and a predefined observation/manipulation pose;
4. plan and execute arm motion;
5. retract the arm and continue the route;
6. record the resulting trajectories and task state transitions.

Perception, LLM/VLM planning, automatic grasp-point estimation, dynamic obstacles, and simultaneous whole-body base-arm control are intentionally out of scope for V0.1.

## Planned stack

- ROS 2 Humble
- Gazebo simulation backend
- Nav2 for mobile-base navigation
- MoveIt 2 for arm motion planning
- ros2_control for simulated actuation
- RViz 2 for operator interaction and visualization

See `docs/superpowers/specs/2026-08-19-v0.1-simulation-baseline-design.md` for the frozen V0.1 design.
