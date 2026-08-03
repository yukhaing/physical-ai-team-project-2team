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
