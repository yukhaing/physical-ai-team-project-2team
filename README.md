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

- `camera_homography_7point_calibration_node.py`: create the current camera calibration
- `box_target_pose_bridge_node.py`: validated PoseStamped target to Cyclo MoveL
- `omx_target_panel`: RViz target-entry and reachable-workspace panel

The camera preview publishes `/camera_box_target`, which is intentionally separate
from the robot command input `/box_target_pose`.

## Integrated system startup

Zenoh, robot bringup, MoveJ, camera, YOLO target bridge, coordinator, RViz, and the
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

The external detector must publish
`std_msgs/msg/Float64MultiArray` on `/yolo/selected_box` using
`[is_defect, confidence, x_link0_m, y_link0_m, joint5_rad]`. The bridge accepts
only a stable, in-workspace defect while the coordinator is waiting for a new
pick target. It forwards X/Y and a stable joint5 target to `/camera_box_target`.
The high-Z XY approach moves X/Y and joint5 together; pitch and descent retain
the achieved gripper angle.

Prepare and run the isolated YOLO detector environment in
`physical_ai_server`:

```bash
./scripts/start_yolo_detector.sh
```

This copies the checked-in model and calibration to `/opt/omx_yolo`, creates a
separate virtual environment with a ROS-compatible NumPy version, and leaves
the existing ACT Python environment unchanged. Check it from another terminal:

```bash
./scripts/status_yolo_detector.sh
```

`./scripts/status_omx_system.sh` also reports the YOLO publisher. The bridge
node alone is not considered a complete target source; `/yolo/selected_box`
must show at least one publisher before robot motion is requested.

For a new physical setup, edit the seven measured `link0` X/Y points in
`src/omx_box_control/config/homography_7point_calibration.yaml`, rebuild the
package, and run:

```bash
ros2 launch omx_box_control camera_homography_7point_calibration.launch.py
```

Press `c` and click all seven points in configuration order. The generated
`omx_camera_homography_7point.yaml` is required by the YOLO runtime setup
script. There is no fallback to an older environment's calibration. This tool only reads images and never
commands the robot. After saving, click independent check points to preview
their transformed `link0` X/Y without publishing a target.
