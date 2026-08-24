#!/usr/bin/env python3
"""Experimental ACT -> OMX direct command runner.

This preserves the temporary integration used during the 2026-08 experiments.
It is intentionally DRY-RUN by default. This is not the production LeRobot
execution pipeline: policy actions are treated as follower joint targets, then
clamped against the latest measured state before publication.
"""

import argparse
import math
import time

import numpy as np
import rclpy
import torch
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5"]
STATE_NAMES = ARM_NAMES + ["gripper_joint_1"]


class DirectActRunner(Node):
    def __init__(self, args):
        super().__init__("experimental_act_direct_clamped_runner")
        self.args = args
        self.bridge = CvBridge()
        self.images = {}
        self.state = None
        self.stamps = {}
        self.steps = 0

        cfg = PreTrainedConfig.from_pretrained(args.model)
        cfg.device = args.device
        self.policy = ACTPolicy.from_pretrained(
            args.model, config=cfg, cache_dir=args.cache_dir
        ).to(args.device)
        self.policy.eval()

        self.arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.gripper = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_cmd"
        )
        self.create_subscription(
            Image, args.wrist_topic, lambda m: self._image_cb("wrist_camera", m), 1
        )
        self.create_subscription(
            Image, args.fixed_topic, lambda m: self._image_cb("fixed_camera", m), 1
        )
        self.create_subscription(JointState, "/joint_states", self._state_cb, 10)
        self.timer = self.create_timer(args.period, self._tick)
        mode = "PUBLISH" if args.publish else "DRY-RUN"
        self.get_logger().warning(f"Experimental direct ACT runner ready: {mode}")

    def _image_cb(self, key, msg):
        image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        self.images[key] = image
        self.stamps[key] = time.monotonic()

    def _state_cb(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in STATE_NAMES):
            self.state = np.asarray([values[name] for name in STATE_NAMES], np.float32)
            self.stamps["state"] = time.monotonic()

    def _fresh(self):
        now = time.monotonic()
        keys = ("wrist_camera", "fixed_camera", "state")
        return all(k in self.stamps and now - self.stamps[k] <= self.args.max_age for k in keys)

    def _tensor_image(self, key):
        image = self.images[key]
        return torch.from_numpy(image.copy()).permute(2, 0, 1).float().div(255).unsqueeze(0)

    def _tick(self):
        if self.steps >= self.args.max_steps or not self._fresh():
            return
        observation = {
            "observation.images.wrist_camera": self._tensor_image("wrist_camera").to(self.args.device),
            "observation.images.fixed_camera": self._tensor_image("fixed_camera").to(self.args.device),
            "observation.state": torch.from_numpy(self.state.copy()).unsqueeze(0).to(self.args.device),
        }
        with torch.inference_mode():
            raw = self.policy.select_action(observation).detach().cpu().numpy().reshape(-1)
        if raw.size != 6 or not np.isfinite(raw).all():
            self.get_logger().error(f"Rejected invalid action: {raw}")
            return

        delta = raw - self.state
        safe = self.state.copy()
        safe[:5] += np.clip(delta[:5], -self.args.arm_delta, self.args.arm_delta)
        safe[5] += np.clip(delta[5], -self.args.gripper_delta, self.args.gripper_delta)
        self.get_logger().info(
            f"step={self.steps} raw={np.round(raw, 4)} safe={np.round(safe, 4)}"
        )
        self.steps += 1
        if not self.args.publish:
            return

        trajectory = JointTrajectory()
        trajectory.joint_names = ARM_NAMES
        point = JointTrajectoryPoint()
        point.positions = safe[:5].tolist()
        duration_ns = int(self.args.command_duration * 1e9)
        point.time_from_start = Duration(
            sec=duration_ns // 1_000_000_000,
            nanosec=duration_ns % 1_000_000_000,
        )
        trajectory.points = [point]
        self.arm_pub.publish(trajectory)

        if self.gripper.server_is_ready() and math.isfinite(float(safe[5])):
            goal = GripperCommand.Goal()
            goal.command.position = float(safe[5])
            goal.command.max_effort = self.args.gripper_effort
            self.gripper.send_goal_async(goal)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="baemseo/omx_box_v1")
    parser.add_argument("--cache-dir", default="/tmp/hf_models")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--wrist-topic", default="/wrist_camera/image_raw")
    parser.add_argument("--fixed-topic", default="/camera1/image_raw")
    parser.add_argument("--period", type=float, default=0.15)
    parser.add_argument("--command-duration", type=float, default=0.15)
    parser.add_argument("--max-age", type=float, default=0.5)
    parser.add_argument("--max-steps", type=int, default=10)
    parser.add_argument("--arm-delta", type=float, default=0.02)
    parser.add_argument("--gripper-delta", type=float, default=0.01)
    parser.add_argument("--gripper-effort", type=float, default=5.0)
    parser.add_argument(
        "--publish",
        action="store_true",
        help="Actually command the robot. Without this flag the runner is dry-run only.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DirectActRunner(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
