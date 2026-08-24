# Experimental ACT direct runner

`act_direct_clamped_runner.py` preserves the temporary ACT integration tested
on the OMX setup. It feeds the two fixed camera streams and six joint states to
`baemseo/omx_box_v1`, interprets the returned six values as direct follower
joint targets, clamps each command against the measured state, and publishes
the arm and gripper commands separately.

This is retained for experiment reproduction and comparison only. It is not
the final LeRobot deployment pipeline. The direct interpretation can diverge
from the action conversion used during data collection.

## Defaults

- dry-run: enabled (no robot command)
- model: `baemseo/omx_box_v1`
- wrist input: `/wrist_camera/image_raw`
- fixed input: `/camera1/image_raw`
- state order: `joint1..joint5, gripper_joint_1`
- arm clamp: `0.02 rad/step`
- gripper clamp: `0.01 rad/step`
- command duration: `0.15 s`
- maximum steps: `10`

## Run inside `physical_ai_server`

```bash
source /opt/ros/jazzy/setup.bash
python3 act_direct_clamped_runner.py
```

The command above only logs raw and clamped actions. Physical publication must
be explicitly enabled:

```bash
python3 act_direct_clamped_runner.py --publish --max-steps 10
```

Before using `--publish`, support the robot, clear the workspace, verify ID12
torque/temperature, verify both camera topics, and keep power removal available.

## Observed limitation

The limited 10-step arm test ran without NaN or stale observations, but a later
direct policy attempt moved diagonally away from the box. Do not use this runner
as evidence that the trained policy is invalid; validate the official LeRobot
action conversion/execution path separately.
