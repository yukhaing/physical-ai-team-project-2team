#!/usr/bin/env python3
"""Confirmed rule-based approach, grasp, and lift pipeline for OMX-F."""
import math
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robotis_interfaces.msg import MoveL
from std_msgs.msg import String
from std_srvs.srv import Trigger


class RuleBasedPick(Node):
    MOTIONS = {'MOVING_STAGING', 'ROTATING_DOWN', 'MOVING_APPROACH', 'DESCENDING', 'LIFTING'}

    def __init__(self):
        super().__init__('rule_based_pick')
        defaults = {'target_topic': '/camera_box_target', 'current_pose_topic': '/omx_movel_controller/current_pose',
                    'movel_topic': '/omx_movel_controller/movel', 'gripper_action': '/gripper_controller/gripper_cmd',
                    'controller_error_topic': '/omx_movel_controller/controller_error',
                    'base_frame': 'link0', 'dry_run': True, 'movej_staging_done': False,
                    'lift_after_grasp': False,
                    'approach_offset': 0.05, 'grasp_offset': 0.0,
                    'lift_offset': 0.05, 'move_duration': 6.0, 'position_tolerance': 0.008,
                    'orientation_tolerance': 0.05,
                    'settle_time': 0.3, 'state_timeout': 12.0, 'target_max_age': 30.0,
                    'max_motion_retries': 2, 'retry_max_error': 0.05,
                    'staging_x': 0.18, 'staging_y': 0.0, 'staging_z': 0.12,
                    'staging_pitch': 1.20,
                    'tool_pitch': 1.25,
                    'open_position': 1.0, 'closed_position': 0.0, 'gripper_effort': 10.0,
                    'min_x': 0.08, 'max_x': 0.32, 'max_abs_y': 0.25, 'min_z': 0.01, 'max_z': 0.32}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self.target = self.current = self.commanded = self.staging = self.stable_since = None
        self.motion_retry_count = 0
        self.state, self.started = 'IDLE', self.get_clock().now()
        self.move_pub = self.create_publisher(MoveL, self.p('movel_topic'), 10)
        self.status_pub = self.create_publisher(String, '/rule_based_pick/status', 10)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, 10)
        self.create_subscription(PoseStamped, self.p('current_pose_topic'), self.on_pose, 10)
        self.create_subscription(String, self.p('controller_error_topic'), self.on_controller_error, 10)
        self.gripper = ActionClient(self, GripperCommand, self.p('gripper_action'))
        self.create_service(Trigger, '~/confirm', self.on_confirm)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.create_timer(0.05, self.update)
        self.report('ready; click a calibrated point, then call ~/confirm')

    def p(self, key): return self.get_parameter(key).value
    def report(self, text):
        msg = String(data=f'{self.state}: {text}'); self.status_pub.publish(msg); self.get_logger().info(msg.data)
    def enter(self, state, text):
        self.state, self.started, self.stable_since = state, self.get_clock().now(), None
        self.motion_retry_count = 0
        self.report(text)
    def on_pose(self, msg): self.current = msg
    def on_controller_error(self, msg):
        if self.state != 'IDLE': self.fail(msg.data)
    def on_target(self, msg):
        if self.state != 'IDLE': return
        if msg.header.frame_id != self.p('base_frame'):
            self.get_logger().warning(f'Ignored target frame {msg.header.frame_id!r}'); return
        self.target = msg; p = msg.pose.position
        self.report(f'target latched x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}')

    def on_confirm(self, _req, res):
        if self.state != 'IDLE': res.success, res.message = False, f'busy: {self.state}'; return res
        if self.current is None: res.success, res.message = False, 'no current pose received'; return res
        error = self.validate()
        if error: res.success, res.message = False, error; return res
        if self.p('dry_run'):
            res.success, res.message = False, 'dry_run=true; all poses valid, commands disabled'; self.report(res.message); return res
        if not self.gripper.server_is_ready(): res.success, res.message = False, 'gripper action unavailable'; return res
        if self.p('movej_staging_done'):
            p = self.current.pose.position
            self.move(self.vertical_pose(p.x, p.y, p.z))
            self.enter('ROTATING_DOWN', 'MoveJ staging accepted; rotating vertically downward')
        else:
            self.staging = self.staging_pose()
            self.move(self.staging)
            self.enter('MOVING_STAGING', 'moving to the shared staging pose')
        res.success, res.message = True, 'pick sequence started'; return res

    def on_cancel(self, _req, res):
        active = self.state != 'IDLE'; self.hold_position(); self.enter('IDLE', 'cancelled; holding current pose')
        res.success, res.message = active, 'cancelled' if active else 'already idle'; return res

    def validate(self):
        if self.target is None: return 'no camera target received'
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(self.target.header.stamp)).nanoseconds / 1e9
        if age > self.p('target_max_age'): return f'target is stale ({age:.1f}s)'
        for dz in (self.p('grasp_offset'), self.p('approach_offset'), self.p('lift_offset')):
            p = self.pose(dz).pose.position
            if not self.safe(p.x, p.y, p.z):
                return f'pose outside workspace: ({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
        if not self.p('movej_staging_done'):
            staging = self.staging_pose().pose.position
            if not self.safe(staging.x, staging.y, staging.z):
                return f'staging pose outside workspace: ({staging.x:.3f}, {staging.y:.3f}, {staging.z:.3f})'
        if not all(math.isfinite(float(self.p(name))) for name in ('staging_pitch', 'tool_pitch')):
            return 'staging_pitch and tool_pitch must be finite'
        return ''

    def safe(self, x, y, z):
        return (all(math.isfinite(v) for v in (x, y, z)) and
                self.p('min_x') <= x <= self.p('max_x') and
                abs(y) <= self.p('max_abs_y') and
                self.p('min_z') <= z <= self.p('max_z'))

    def pose(self, dz):
        p = self.target.pose.position
        return self.vertical_pose(p.x, p.y, p.z + dz)

    def vertical_pose(self, x, y, z):
        return self.oriented_pose(x, y, z, float(self.p('tool_pitch')))

    def oriented_pose(self, x, y, z, pitch):
        out = PoseStamped(); out.header.stamp = self.get_clock().now().to_msg(); out.header.frame_id = self.p('base_frame')
        out.pose.position.x, out.pose.position.y, out.pose.position.z = x, y, z
        yaw = math.atan2(y, x + 0.01125)
        half_yaw, half_pitch = yaw / 2.0, pitch / 2.0
        # Rz(yaw) * Ry(pitch): keep the gripper radial while pitching its +X tool axis down.
        out.pose.orientation.x = -math.sin(half_yaw) * math.sin(half_pitch)
        out.pose.orientation.y = math.cos(half_yaw) * math.sin(half_pitch)
        out.pose.orientation.z = math.sin(half_yaw) * math.cos(half_pitch)
        out.pose.orientation.w = math.cos(half_yaw) * math.cos(half_pitch)
        return out

    def staging_pose(self):
        return self.oriented_pose(
            float(self.p('staging_x')), float(self.p('staging_y')),
            float(self.p('staging_z')), float(self.p('staging_pitch')))

    def move(self, pose):
        msg = MoveL(); msg.pose = pose; duration = max(0.02, self.p('move_duration'))
        msg.time_from_start.sec, msg.time_from_start.nanosec = int(duration), int((duration % 1) * 1e9)
        self.move_pub.publish(msg); self.commanded = pose

    def hold_position(self):
        if self.current is None:
            self.commanded = None; return
        msg = MoveL(); msg.pose = self.current
        self.move_pub.publish(msg); self.commanded = None

    def grip(self, position):
        goal = GripperCommand.Goal(); goal.command.position, goal.command.max_effort = position, self.p('gripper_effort')
        self.gripper.send_goal_async(goal).add_done_callback(self.on_grip_goal)
    def on_grip_goal(self, future):
        handle = future.result()
        if not handle.accepted: self.fail('gripper goal rejected'); return
        handle.get_result_async().add_done_callback(self.on_grip_result)
    def on_grip_result(self, future):
        try:
            wrapped = future.result()
        except Exception as error:
            self.fail(f'gripper result failed: {error}')
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self.fail(f'gripper action status={wrapped.status}')
            return
        result = wrapped.result
        if not result.reached_goal:
            self.fail(
                f'gripper did not reach goal; position={result.position:.4f}, '
                f'effort={result.effort:.2f}, stalled={result.stalled}')
            return
        if self.state == 'OPENING_GRIPPER': self.move(self.pose(self.p('grasp_offset'))); self.enter('DESCENDING', 'descending')
        elif self.state == 'CLOSING_GRIPPER':
            if self.p('lift_after_grasp'):
                self.move(self.pose(self.p('lift_offset'))); self.enter('LIFTING', 'lifting')
            else:
                self.commanded = None
                self.enter('IDLE', 'GRASP_COMPLETED: gripper closed; waiting for MoveJ handoff')

    def update(self):
        if self.state == 'IDLE': return
        if (self.get_clock().now() - self.started).nanoseconds / 1e9 > self.p('state_timeout'):
            error = self.motion_error()
            if (self.state in self.MOTIONS and error is not None and
                    error <= self.p('retry_max_error') and
                    self.motion_retry_count < self.p('max_motion_retries')):
                self.motion_retry_count += 1
                retry = self.motion_retry_count
                maximum = self.p('max_motion_retries')
                goal = self.commanded
                self.started, self.stable_since = self.get_clock().now(), None
                self.move(goal)
                self.report(f'refining target, retry {retry}/{maximum}; error={error*1000:.1f}mm')
                return
            self.fail(f'timeout in {self.state}; {self.motion_error_text()}'); return
        if self.state not in self.MOTIONS or not self.motion_done(): return
        if self.state == 'MOVING_STAGING':
            p = self.staging.pose.position
            self.move(self.vertical_pose(p.x, p.y, p.z))
            self.enter('ROTATING_DOWN', 'rotating gripper vertically downward at safe height')
        elif self.state == 'ROTATING_DOWN':
            self.move(self.pose(self.p('approach_offset')))
            self.enter('MOVING_APPROACH', 'moving horizontally to approach pose')
        elif self.state == 'MOVING_APPROACH': self.commanded = None; self.enter('OPENING_GRIPPER', 'opening'); self.grip(self.p('open_position'))
        elif self.state == 'DESCENDING': self.commanded = None; self.enter('CLOSING_GRIPPER', 'closing'); self.grip(self.p('closed_position'))
        else: self.commanded = None; self.enter('IDLE', 'pick completed')

    def motion_done(self):
        if self.current is None or self.commanded is None: return False
        a, g = self.current.pose.position, self.commanded.pose.position
        if math.sqrt((a.x-g.x)**2 + (a.y-g.y)**2 + (a.z-g.z)**2) > self.p('position_tolerance'):
            self.stable_since = None; return False
        if self.orientation_error() > self.p('orientation_tolerance'):
            self.stable_since = None; return False
        now = self.get_clock().now()
        if self.stable_since is None: self.stable_since = now; return False
        return (now - self.stable_since).nanoseconds / 1e9 >= self.p('settle_time')
    def motion_error(self):
        if self.current is None or self.commanded is None: return None
        a, g = self.current.pose.position, self.commanded.pose.position
        return math.sqrt((a.x-g.x)**2 + (a.y-g.y)**2 + (a.z-g.z)**2)
    def motion_error_text(self):
        error = self.motion_error()
        if error is None: return 'pose feedback unavailable'
        a, g = self.current.pose.position, self.commanded.pose.position
        angle = self.orientation_error()
        return (f'goal=({g.x:.3f},{g.y:.3f},{g.z:.3f}), '
                f'actual=({a.x:.3f},{a.y:.3f},{a.z:.3f}), error={error*1000:.1f}mm, '
                f'orientation_error={math.degrees(angle):.1f}deg')
    def orientation_error(self):
        if self.current is None or self.commanded is None: return math.inf
        a, g = self.current.pose.orientation, self.commanded.pose.orientation
        dot = abs(a.x*g.x + a.y*g.y + a.z*g.z + a.w*g.w)
        return 2.0 * math.acos(max(0.0, min(1.0, dot)))
    def fail(self, reason): self.hold_position(); self.enter('IDLE', f'FAILED: {reason}')


def main(args=None):
    rclpy.init(args=args); node = RuleBasedPick()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__ == '__main__': main()
