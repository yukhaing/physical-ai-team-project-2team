# OMX Box Project

Independent ROS 2 overlay workspace for camera-based OMX-F box targeting.

## Start the independent development container

The project uses `robotis/open-manipulator:5.0.0` as its ROS/OMX base image and
builds the official `ROBOTIS-GIT/cyclo_control` source into the image. It does
not mount or build from the host `~/open_manipulator` checkout.

```bash
cd ~/omx_box_project_ws/docker
./container.sh start
./container.sh enter
```

Do not run the original `open_manipulator` container at the same time when a
physical OMX-F is connected; both containers would have access to the devices.

## Build inside the project container

```bash
source /opt/ros/jazzy/setup.bash
cd /root/omx_box_project_ws
colcon build --symlink-install
source install/setup.bash
```

Cyclo physical configuration is available at:

```text
/root/omx_box_project_ws/docker/config/omx_config_physical.yaml
```

## Nodes

- `camera_homography_target_node.py`: webcam click to preview pose and RViz marker
- `box_target_pose_bridge_node.py`: validated PoseStamped target to Cyclo MoveL
- `omx_target_panel`: RViz target-entry and reachable-workspace panel

The camera preview publishes `/camera_box_target`, which is intentionally separate
from the robot command input `/box_target_pose`.

## Integrated system startup

Zenoh, robot bringup, MoveJ, camera, homography, coordinator, RViz, and the
status monitor run in separate `tmux` windows inside the Docker container.
The start command prepares the nodes only; it does not request robot motion.

```bash
./scripts/start_omx_system.sh
./scripts/attach_omx_system.sh
./scripts/status_omx_system.sh
./scripts/stop_omx_system.sh
```

Inside `tmux`, use `Ctrl-b` followed by `n`/`p` to change windows and
`Ctrl-b d` to detach. After checking the physical workspace, begin staging
explicitly:

```bash
docker exec -it omx_box_project bash -lc '
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /root/omx_box_project_ws/install/setup.bash
ros2 service call /pick_coordinator/start std_srvs/srv/Trigger "{}"
'
```

The defaults are `/dev/ttyACM0` and `/dev/video0`. Override them when needed:

```bash
OMX_PORT_NAME=/dev/ttyACM1 OMX_VIDEO_DEVICE=/dev/video2 \
  ./scripts/start_omx_system.sh
```

Use the team's calibrated YOLO output instead of manual homography clicks:

```bash
OMX_TARGET_SOURCE=yolo ./scripts/start_omx_system.sh
```

In YOLO mode, the external detector must publish
`std_msgs/msg/Float64MultiArray` on `/yolo/selected_box` using
`[is_defect, confidence, x_link0_m, y_link0_m, joint5_rad]`. The bridge accepts
only a stable, in-workspace defect while the coordinator is waiting for a new
pick target. It forwards X/Y to `/camera_box_target`; detected joint5 is logged
but intentionally not commanded until it has separate physical validation.
