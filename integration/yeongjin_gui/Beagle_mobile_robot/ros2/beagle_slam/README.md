# Beagle SLAM map creation

This package bridges the Robomation Beagle to ROS 2 Jazzy and starts SLAM
Toolbox. It publishes `/scan`, `/odom`, the required TF tree, and accepts
`/cmd_vel` for slow manual mapping.

Do not run the legacy shuttle mission or `gui-up` while mapping. The Robomation
USB receiver can only be owned by one Beagle process.

## One-time preparation

From the repository root:

```bash
./scripts/beagle_mapping_container.sh install-deps
./scripts/beagle_mapping_container.sh build
```

The dependency install modifies the currently running container. Add
`ros-jazzy-nav2-map-server`, `ros-jazzy-slam-toolbox`, and
`ros-jazzy-teleop-twist-keyboard` to `docker/Dockerfile` before rebuilding the
image if the setup must persist across container recreation.

## Required calibration

Edit `config/beagle_base.yaml` before trusting a map:

- Measure `laser_x_m`, `laser_y_m`, and `laser_z_m` from the midpoint of the
  drive-wheel axle to the LiDAR centre.
- Verify `meters_per_encoder_unit` by resetting the encoders, rolling the robot
  straight by a measured distance, and comparing both encoder changes.
- If forward rolling makes either encoder decrease, change that wheel's
  `encoder_*_sign` to `-1.0`.

The Robomation API documents LiDAR indices as 0 degrees front, 90 degrees left,
180 degrees rear and 270 degrees right. It documents distances in millimetres,
but does not state a physical unit for the encoder count. The checked-in
`0.001 m/unit` value is only a calibration starting point.

## Map the workcell

Place the stopped robot at the position that should become the map origin.
Then run:

```bash
./scripts/beagle_mapping_container.sh start
./scripts/beagle_mapping_container.sh status
./scripts/beagle_mapping_container.sh teleop
```

Drive slowly, observe the map in RViz, cover the whole boundary, and finish near
the starting pose so SLAM Toolbox can close the loop. Save and stop from another
terminal:

```bash
./scripts/beagle_mapping_container.sh save workcell
./scripts/beagle_mapping_container.sh stop
```

The occupancy map is saved as `maps/workcell.yaml` and `maps/workcell.pgm`.
The reusable SLAM pose graph is saved as `maps/workcell.posegraph` and
`maps/workcell.data`.
