# Experimental ACT direct runner

This directory preserves the temporary ACT-to-OMX integration tested during
development. It is kept for reproduction and comparison, not as the final
LeRobot deployment path.

The runner supplies the two unchanged fixed-camera streams and six measured
joint states to `baemseo/omx_box_v1`. It then treats the six policy values as
direct follower targets, clamps them against the latest state, and publishes
arm and gripper commands separately.

## Safety defaults

- No commands are sent unless `--publish` is explicitly supplied.
- Arm change is limited to `0.02 rad/step`.
- Gripper change is limited to `0.01 rad/step`.
- Each trajectory point uses `time_from_start=0.15 s`.
- Execution stops after 10 steps by default.
- Camera and state data older than 0.5 seconds are rejected.

## Usage in `physical_ai_server`

Dry-run:

```bash
source /opt/ros/jazzy/setup.bash
python3 act_direct_clamped_runner.py
```

Physical test (experimental and potentially unsafe):

```bash
python3 act_direct_clamped_runner.py --publish --max-steps 10
```

Before physical publication, support the robot, clear the workspace, verify
ID12 temperature/torque and both camera topics, and keep emergency power removal
available.

## Known result

The limited 10-step arm test produced finite actions with fresh observations.
A later direct-policy attempt moved diagonally away from the box. This does not
show that the trained ACT policy is invalid; the official LeRobot action
conversion and execution pipeline must be validated separately.
