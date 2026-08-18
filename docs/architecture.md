# Architecture

The project separates scene truth, mission semantics, navigation, manipulation, simulation, and operator interaction.

```text
scene/world data
      |
      v
 scene server ------> operator UI
      |
      v
 mission core <------ mission manager
   |                       |
   |                       +--> navigation adapter --> Nav2
   |                       +--> manipulation adapter --> MoveIt 2
   |
   +-----------------------------------------------> recorder

Gazebo supplies simulated physics and ros2_control-facing devices.
```

The mission core is dependency-free and testable without ROS. Navigation and manipulation implementations depend on its interfaces, never the reverse.
