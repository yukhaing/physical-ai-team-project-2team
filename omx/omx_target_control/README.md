# OMX Target Control

ROS 2 Jazzy package for entering a Cartesian target in RViz2 and sending a
validated `robotis_interfaces/msg/MoveL` command to an OMX-F controller.

## Features

- RViz2 panel for direct X/Y/Z input in the `link0` frame
- target axes and approximate reachable-workspace visualization
- bounding-box and radial-reach safety checks
- `/box_target_pose` to `/omx_movel_controller/movel` command bridge

The visualization is an operational safety envelope, not an exact
inverse-kinematics guarantee. The controller can still reject targets that
violate joint, collision, or solver constraints.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select omx_target_control --symlink-install
source install/setup.bash
```

## Run

Start the OMX-F hardware and MoveL controller, then run:

```bash
ros2 run omx_target_control box_target_pose_bridge_node.py
```

In RViz2, select `Panels` → `Add New Panel` →
`omx_target_control/OmxTargetPanel`. Add a `MarkerArray` display with topic
`/box_target_marker`, enter a target, and press **Move**.

## Default safety envelope

- X: 0.08–0.32 m
- Y: -0.25–0.25 m
- Z: 0.01–0.32 m
- radial distance from `link0`: 0.10–0.42 m

Bridge limits are ROS parameters and may be adjusted after calibration.
