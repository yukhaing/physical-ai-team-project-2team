#!/usr/bin/env python3
"""Feedback-gated coordinator for the validated all-MoveJ box cycle."""

from functools import partial
import math

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory


class PickCoordinator(Node):
    """Run each proven node in order, advancing only from measured success."""

    IDLE = 'IDLE'
    WAIT_STAGING = 'WAIT_STAGING'
    WAIT_STAGING_CLOSE = 'WAIT_STAGING_GRIPPER_CLOSE'
    WAIT_PICK_TARGET = 'WAIT_PICK_TARGET'
    WAIT_XY = 'WAIT_XY_APPROACH'
    WAIT_PITCH = 'WAIT_PITCH_PREGRASP'
    WAIT_PICK_OPEN = 'WAIT_PICK_GRIPPER_OPEN'
    WAIT_PICK_DESCENT = 'WAIT_PICK_DESCENT'
    WAIT_PICK_CLOSE = 'WAIT_PICK_GRIPPER_CLOSE'
    WAIT_GRASP_CONFIRM = 'WAIT_GRASP_CONFIRM'
    WAIT_LIFT = 'WAIT_LOADED_LIFT'
    WAIT_PLACE_XY_TRANSFER = 'WAIT_PLACE_HIGH_XY_TRANSFER'
    WAIT_PLACE_RECOVERY = 'WAIT_PLACE_XY_PITCH_APPROACH'
    WAIT_PLACE_ROTATE = 'WAIT_PLACE_ROTATE'
    WAIT_PLACE_CORRECTION = 'WAIT_PLACE_LIFT_CORRECTION'
    WAIT_PLACE_DESCENT = 'WAIT_PLACE_DESCENT'
    WAIT_PLACE_OPEN = 'WAIT_PLACE_GRIPPER_OPEN'
    WAIT_PLACE_RETRACT = 'WAIT_PLACE_RETRACT'
    WAIT_RETURN_STAGING = 'WAIT_RETURN_STAGING'
    WAIT_FINAL_CLOSE = 'WAIT_FINAL_GRIPPER_CLOSE'
    COMPLETE = 'COMPLETE'
    FAILED = 'FAILED'

    def __init__(self):
        super().__init__('pick_coordinator')
        defaults = {
            'movej_topic': '/omx_movej_controller/movej',
            'staging_command_topic': '/pick_coordinator/commands/staging',
            'xy_command_topic': '/pick_coordinator/commands/xy',
            'pitch_command_topic': '/pick_coordinator/commands/pitch',
            'pick_descent_command_topic': '/pick_coordinator/commands/pick_descent',
            'lift_command_topic': '/pick_coordinator/commands/lift',
            'place_xy_transfer_command_topic': '/pick_coordinator/commands/place_xy_transfer',
            'place_recovery_command_topic': '/pick_coordinator/commands/place_recovery',
            'place_rotate_command_topic': '/pick_coordinator/commands/place_rotate',
            'place_correction_command_topic': '/pick_coordinator/commands/place_correction',
            'place_descent_command_topic': '/pick_coordinator/commands/place_descent',
            'joint_states_topic': '/joint_states',
            'target_topic': '/camera_box_target',
            'base_frame': 'link0',
            'feedback_timeout': 0.5,
            'target_max_age': 30.0,
            'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'],
            'gripper_joint_name': 'gripper_joint_1',
            'gripper_action': '/gripper_controller/gripper_cmd',
            'gripper_open_position': 1.0,
            'gripper_closed_position': 0.0,
            'gripper_effort': 10.0,
            'minimum_open_position': 0.80,
            'open_tolerance': 0.05,
            'open_watchdog_timeout': 2.0,
            'open_watchdog_stable_time': 0.50,
            'open_watchdog_position_epsilon': 0.003,
            'maximum_staging_closed_position': 0.05,
            'min_grasp_position': 0.005,
            'max_grasp_position': 0.60,
            'grasp_watchdog_timeout': 3.0,
            'grasp_watchdog_stable_time': 0.50,
            'grasp_watchdog_position_epsilon': 0.003,
            'empty_close_watchdog_timeout': 2.0,
            'empty_close_watchdog_stable_time': 0.30,
            'require_grasp_confirmation': True,
            'recover_grasp_min_z': 0.045,
            'recover_grasp_max_z': 0.070,
            'recover_lift_min_z': 0.110,
            'recover_lift_max_z': 0.150,
            'place_correction_trigger_z': 0.120,
            'remembered_place_xyz': [0.1711, -0.2197, 0.1236],
            'remembered_place_release_xyz': [0.1711, -0.2197, 0.0500],
            'place_target_topic': '/pick_coordinator/place_target',
            'place_release_target_topic': '/pick_coordinator/place_release_target',
            'fk_position_bias': [0.00126, 0.0, 0.00055],
        }
        services = {
            'staging': '/movej_staging/confirm',
            'xy': '/movej_xy_approach/confirm',
            'pitch': '/movej_pitch_pregrasp/confirm',
            'pick_open': '/gripper_open/confirm',
            'pick_descent': '/movej_single_ptp_descent/confirm',
            'lift': '/movej_lift/confirm',
            'place_xy_transfer': '/movej_place_xy_transfer/confirm',
            'place_recovery': '/movej_place_recovery/confirm',
            'place_rotate': '/movej_place_rotate/confirm',
            'place_correction': '/movej_place_lift_correction/confirm',
            'place_descent': '/movej_single_ptp_place_descent/confirm',
        }
        statuses = {
            'staging': '/movej_staging/status',
            'xy': '/movej_xy_approach/status',
            'pitch': '/movej_pitch_pregrasp/status',
            'pick_open': '/gripper_open/status',
            'pick_descent': '/movej_single_ptp_descent/status',
            'lift': '/movej_lift/status',
            'place_xy_transfer': '/movej_place_xy_transfer/status',
            'place_recovery': '/movej_place_recovery/status',
            'place_rotate': '/movej_place_rotate/status',
            'place_correction': '/movej_place_lift_correction/status',
            'place_descent': '/movej_single_ptp_place_descent/status',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        for key, value in services.items():
            self.declare_parameter(f'{key}_service', value)
        for key, value in statuses.items():
            self.declare_parameter(f'{key}_status_topic', value)

        self.state = self.IDLE
        self.pending_call = False
        self.pending_gripper = False
        self.gripper_goal_handle = None
        self.grasp_watchdog_started = None
        self.grasp_watchdog_stable_since = None
        self.grasp_watchdog_last_position = None
        self.grasp_watchdog_accept_cancel = False
        self.grasp_watchdog_cancel_requested = False
        self.open_watchdog_started = None
        self.open_watchdog_stable_since = None
        self.open_watchdog_last_position = None
        self.open_watchdog_accept_cancel = False
        self.open_watchdog_cancel_requested = False
        self.positions = {}
        self.joint_feedback_time = None
        self.target = None
        self.target_received_time = None
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.place_target_pub = self.create_publisher(
            PoseStamped, self.p('place_target_topic'), 10)
        self.place_release_target_pub = self.create_publisher(
            PoseStamped, self.p('place_release_target_topic'), 10)
        # This is intentionally the only publisher connected to the physical
        # MoveJ input. Stage nodes publish to isolated internal topics.
        self.movej_pub = self.create_publisher(JointTrajectory, self.p('movej_topic'), 10)
        self.stage_clients = {
            key: self.create_client(Trigger, self.p(f'{key}_service'))
            for key in services
        }
        for key in statuses:
            self.create_subscription(
                String, self.p(f'{key}_status_topic'),
                partial(self.on_stage_status, key), 10)
        command_states = {
            'staging': (self.WAIT_STAGING, self.WAIT_RETURN_STAGING),
            'xy': (self.WAIT_XY,),
            'pitch': (self.WAIT_PITCH,),
            'pick_descent': (self.WAIT_PICK_DESCENT,),
            'lift': (self.WAIT_LIFT,),
            'place_xy_transfer': (
                self.WAIT_PLACE_XY_TRANSFER, self.WAIT_PLACE_RETRACT),
            'place_recovery': (self.WAIT_PLACE_RECOVERY,),
            'place_rotate': (self.WAIT_PLACE_ROTATE,),
            'place_correction': (self.WAIT_PLACE_CORRECTION,),
            'place_descent': (self.WAIT_PLACE_DESCENT,),
        }
        for key, allowed_states in command_states.items():
            self.create_subscription(
                JointTrajectory, self.p(f'{key}_command_topic'),
                partial(self.on_stage_command, key, allowed_states), 10)
        self.create_subscription(JointState, self.p('joint_states_topic'), self.on_joints, 10)
        self.create_subscription(PoseStamped, self.p('target_topic'), self.on_target, 10)
        self.gripper = ActionClient(self, GripperCommand, self.p('gripper_action'))
        self.create_service(Trigger, '~/start', self.on_start)
        self.create_service(Trigger, '~/continue', self.on_continue)
        self.create_service(Trigger, '~/recover_grasp', self.on_recover_grasp)
        self.create_service(Trigger, '~/recover_lift', self.on_recover_lift)
        self.create_service(Trigger, '~/cancel', self.on_cancel)
        self.create_timer(0.05, self.update_grasp_watchdog)
        self.create_timer(0.05, self.update_open_watchdog)
        self.report('ready; MoveJ only, no other MoveJ publishers, then call ~/start')

    def p(self, name):
        return self.get_parameter(name).value

    def report(self, text):
        message = String(data=f'{self.state}: {text}')
        self.status_pub.publish(message)
        self.get_logger().info(message.data)

    def enter(self, state, text):
        self.state = state
        self.report(text)

    def fail(self, reason):
        self.pending_call = False
        self.pending_gripper = False
        self.enter(self.FAILED, f'{reason}; no further command sent')

    def on_joints(self, message):
        self.positions.update(zip(message.name, message.position))
        self.joint_feedback_time = self.get_clock().now()

    def on_target(self, message):
        if message.header.frame_id != self.p('base_frame'):
            return
        self.target = message
        self.target_received_time = self.get_clock().now()
        if self.state == self.WAIT_PICK_TARGET:
            point = message.pose.position
            orientation = message.pose.orientation
            joint5 = 2.0 * math.atan2(orientation.x, orientation.w)
            self.report(
                f'pick target received x={point.x:.4f}, y={point.y:.4f}, '
                f'joint5={math.degrees(joint5):.2f}deg; '
                'call ~/continue to begin uninterrupted pick')

    def on_stage_command(self, key, allowed_states, message):
        if self.state not in allowed_states:
            self.fail(f'blocked unexpected {key} command while in {self.state}')
            return
        if not message.points:
            self.fail(f'blocked empty {key} trajectory')
            return
        self.movej_pub.publish(message)
        self.report(f'forwarded one validated {key} trajectory')

    def preflight_error(self):
        publishers = self.get_publishers_info_by_topic(self.p('movej_topic'))
        foreign = [p.node_name for p in publishers if p.node_name != self.get_name()]
        if foreign:
            return f'MoveJ command publisher already active: {foreign}'
        if self.joint_feedback_time is None:
            return 'joint feedback unavailable'
        age = (self.get_clock().now() - self.joint_feedback_time).nanoseconds / 1e9
        if age > float(self.p('feedback_timeout')):
            return f'joint feedback stale ({age:.2f}s)'
        names = list(self.p('joint_names')) + [self.p('gripper_joint_name')]
        if not all(name in self.positions for name in names):
            return 'joint or gripper feedback incomplete'
        return ''

    def fresh_target_error(self):
        if self.target is None or self.target_received_time is None:
            return 'select a pick target first'
        age = (self.get_clock().now() - self.target_received_time).nanoseconds / 1e9
        if age > float(self.p('target_max_age')):
            return f'pick target is stale ({age:.1f}s); select it again'
        return ''

    def request(self, key, next_state, description):
        if self.pending_call or self.pending_gripper:
            return False, 'another request is pending'
        client = self.stage_clients[key]
        if not client.service_is_ready():
            return False, f'{description} service unavailable'
        self.pending_call = True
        self.enter(next_state, f'requesting {description}')
        future = client.call_async(Trigger.Request())
        future.add_done_callback(partial(self.on_request_done, key, next_state, description))
        return True, f'{description} requested'

    def on_request_done(self, key, next_state, description, future):
        self.pending_call = False
        try:
            result = future.result()
        except Exception as error:
            self.fail(f'{description} service error: {error}')
            return
        if not result.success:
            self.fail(f'{description} rejected: {result.message}')
            return
        if key == 'place_rotate' and 'already aligned' in result.message:
            self.after_place_rotation()
            return
        self.report(result.message)

    def on_start(self, _request, response):
        if self.state not in (self.IDLE, self.COMPLETE, self.FAILED):
            response.success, response.message = False, f'busy: {self.state}'
            return response
        error = self.preflight_error()
        if error:
            response.success, response.message = False, error
            return response
        self.target = None
        self.target_received_time = None
        response.success, response.message = self.request(
            'staging', self.WAIT_STAGING, 'staging')
        return response

    def on_recover_grasp(self, _request, response):
        if self.state not in (self.IDLE, self.FAILED):
            response.success, response.message = False, f'cannot recover from {self.state}'
            return response
        error = self.preflight_error()
        if error:
            response.success, response.message = False, error
            return response
        gripper = self.positions.get(self.p('gripper_joint_name'))
        if not (float(self.p('min_grasp_position')) <= gripper <=
                float(self.p('max_grasp_position'))):
            response.success, response.message = False, f'gripper is outside grasp range: {gripper}'
            return response
        z = self.actual_tcp_z()
        if z is None or not (float(self.p('recover_grasp_min_z')) <= z <=
                             float(self.p('recover_grasp_max_z'))):
            response.success, response.message = False, f'TCP Z is outside grasp recovery range: {z}'
            return response
        self.enter(self.WAIT_GRASP_CONFIRM,
                   f'recovered measured grasp at Z={z:.4f}m, gripper={gripper:.4f}rad')
        response.success, response.message = True, 'grasp state recovered; call ~/continue after inspection'
        return response

    def on_recover_lift(self, _request, response):
        if self.state not in (self.IDLE, self.FAILED):
            response.success, response.message = False, f'cannot recover from {self.state}'
            return response
        error = self.preflight_error()
        if error:
            response.success, response.message = False, error
            return response
        gripper = self.positions.get(self.p('gripper_joint_name'))
        if not (float(self.p('min_grasp_position')) <= gripper <=
                float(self.p('max_grasp_position'))):
            response.success, response.message = False, f'gripper is outside grasp range: {gripper}'
            return response
        z = self.actual_tcp_z()
        if z is None or not (float(self.p('recover_lift_min_z')) <= z <=
                             float(self.p('recover_lift_max_z'))):
            response.success, response.message = False, f'TCP Z is outside lift recovery range: {z}'
            return response
        self.publish_place_target()
        response.success, response.message = self.request(
            'place_xy_transfer', self.WAIT_PLACE_XY_TRANSFER,
            'recovered loaded high-Z place XY transfer')
        return response

    def on_continue(self, _request, response):
        if self.state == self.WAIT_PICK_TARGET:
            error = self.preflight_error() or self.fresh_target_error()
            if error:
                response.success, response.message = False, error
                return response
            response.success, response.message = self.request(
                'xy', self.WAIT_XY, 'XY approach')
            return response
        if self.state == self.WAIT_GRASP_CONFIRM:
            gripper = self.positions.get(self.p('gripper_joint_name'))
            if gripper is None or not (
                    float(self.p('min_grasp_position')) <= gripper <=
                    float(self.p('max_grasp_position'))):
                response.success, response.message = False, (
                    f'gripper is outside grasp range: {gripper}')
                return response
            response.success, response.message = self.request(
                'lift', self.WAIT_LIFT, 'loaded lift')
            return response
        response.success, response.message = False, f'continue is not valid in {self.state}'
        return response

    def on_cancel(self, _request, response):
        active = self.state not in (self.IDLE, self.COMPLETE, self.FAILED)
        if self.pending_call or self.pending_gripper:
            response.success, response.message = False, (
                'a command is active; cancel the stage node/action directly and inspect the robot')
            return response
        self.enter(self.IDLE, 'cancelled while idle between commands')
        response.success, response.message = active, 'cancelled' if active else 'already idle'
        return response

    def on_stage_status(self, key, message):
        expected = {
            'staging': (self.WAIT_STAGING, self.WAIT_RETURN_STAGING),
            'xy': (self.WAIT_XY,), 'pitch': (self.WAIT_PITCH,),
            'pick_open': (self.WAIT_PICK_OPEN,),
            'pick_descent': (self.WAIT_PICK_DESCENT,),
            'lift': (self.WAIT_LIFT,),
            'place_xy_transfer': (
                self.WAIT_PLACE_XY_TRANSFER, self.WAIT_PLACE_RETRACT),
            'place_recovery': (self.WAIT_PLACE_RECOVERY,),
            'place_rotate': (self.WAIT_PLACE_ROTATE,),
            'place_correction': (self.WAIT_PLACE_CORRECTION,),
            'place_descent': (self.WAIT_PLACE_DESCENT,),
        }
        if self.state not in expected[key]:
            return
        text = message.data
        if 'FAILED' in text:
            self.fail(f'{key}: {text}')
            return
        if key == 'place_rotate' and text.startswith('COMPLETED STEP:'):
            ok, result = self.request(
                'place_rotate', self.WAIT_PLACE_ROTATE, 'next place rotation step')
            if not ok:
                self.fail(result)
            return
        completed = (
            text.startswith('COMPLETED:') or
            (key in ('pick_descent', 'place_descent') and
             text.startswith('SINGLE_PTP COMPLETED:')))
        if not completed:
            return
        transitions = {
            'xy': ('pitch', self.WAIT_PITCH, 'pitch pregrasp'),
            'pitch': ('pick_open', self.WAIT_PICK_OPEN, 'pick gripper open'),
            'pick_open': ('pick_descent', self.WAIT_PICK_DESCENT, 'pick descent'),
            'lift': ('place_xy_transfer', self.WAIT_PLACE_XY_TRANSFER,
                     'loaded high-Z place XY transfer'),
            'place_recovery': ('place_descent', self.WAIT_PLACE_DESCENT,
                               'place descent'),
            'place_correction': ('place_descent', self.WAIT_PLACE_DESCENT, 'place descent'),
        }
        if key == 'staging':
            if self.state == self.WAIT_STAGING:
                self.command_gripper(
                    False, self.WAIT_STAGING_CLOSE, 'staging gripper close')
            else:
                self.command_gripper(False, self.WAIT_FINAL_CLOSE, 'final gripper close')
            return
        if key == 'pick_descent':
            self.command_gripper(False, self.WAIT_PICK_CLOSE, 'pick gripper close')
            return
        if key == 'place_xy_transfer':
            if self.state == self.WAIT_PLACE_RETRACT:
                ok, result = self.request(
                    'staging', self.WAIT_RETURN_STAGING,
                    'return staging after place retract')
                if not ok:
                    self.fail(result)
                return
            self.publish_place_release_target()
            ok, result = self.request(
                'place_descent', self.WAIT_PLACE_DESCENT,
                'automatic single PTP place descent')
            if not ok:
                self.fail(result)
            return
        if key == 'place_descent':
            self.command_gripper(True, self.WAIT_PLACE_OPEN, 'place gripper open')
            return
        if key in transitions:
            if key == 'lift':
                self.publish_place_target()
            next_key, state, description = transitions[key]
            ok, result = self.request(next_key, state, description)
            if not ok:
                self.fail(result)

    def command_gripper(self, opening, next_state, description):
        if self.pending_call or self.pending_gripper:
            self.fail(f'cannot start {description}: another request is pending')
            return
        if not self.gripper.server_is_ready():
            self.fail(f'{description} action unavailable')
            return
        self.pending_gripper = True
        self.gripper_goal_handle = None
        self.grasp_watchdog_started = None
        self.grasp_watchdog_stable_since = None
        self.grasp_watchdog_last_position = None
        self.grasp_watchdog_accept_cancel = False
        self.grasp_watchdog_cancel_requested = False
        self.open_watchdog_started = None
        self.open_watchdog_stable_since = None
        self.open_watchdog_last_position = None
        self.open_watchdog_accept_cancel = False
        self.open_watchdog_cancel_requested = False
        if not opening and next_state in (
                self.WAIT_PICK_CLOSE, self.WAIT_STAGING_CLOSE,
                self.WAIT_FINAL_CLOSE):
            self.grasp_watchdog_started = self.get_clock().now()
            self.grasp_watchdog_last_position = self.positions.get(
                self.p('gripper_joint_name'))
        if opening:
            self.open_watchdog_started = self.get_clock().now()
            self.open_watchdog_last_position = self.positions.get(
                self.p('gripper_joint_name'))
        goal = GripperCommand.Goal()
        goal.command.position = float(self.p(
            'gripper_open_position' if opening else 'gripper_closed_position'))
        goal.command.max_effort = float(self.p('gripper_effort'))
        future = self.gripper.send_goal_async(goal)
        future.add_done_callback(partial(
            self.on_gripper_goal, opening, next_state, description))
        self.enter(next_state, f'{description} requested')

    def on_gripper_goal(self, opening, next_state, description, future):
        try:
            handle = future.result()
        except Exception as error:
            self.fail(f'{description} goal error: {error}')
            return
        if not handle.accepted:
            self.fail(f'{description} goal rejected')
            return
        self.gripper_goal_handle = handle
        handle.get_result_async().add_done_callback(partial(
            self.on_gripper_result, opening, next_state, description))

    def update_grasp_watchdog(self):
        close_states = (
            self.WAIT_PICK_CLOSE, self.WAIT_STAGING_CLOSE, self.WAIT_FINAL_CLOSE)
        if (not self.pending_gripper or self.state not in close_states or
                self.grasp_watchdog_started is None or
                self.grasp_watchdog_cancel_requested):
            return
        position = self.positions.get(self.p('gripper_joint_name'))
        if position is None:
            return
        now = self.get_clock().now()
        epsilon = float(self.p('grasp_watchdog_position_epsilon'))
        if (self.grasp_watchdog_last_position is None or
                abs(position - self.grasp_watchdog_last_position) > epsilon):
            self.grasp_watchdog_last_position = position
            self.grasp_watchdog_stable_since = now
            return
        if self.grasp_watchdog_stable_since is None:
            self.grasp_watchdog_stable_since = now
            return
        elapsed = (now - self.grasp_watchdog_started).nanoseconds / 1e9
        stable = (now - self.grasp_watchdog_stable_since).nanoseconds / 1e9
        picking = self.state == self.WAIT_PICK_CLOSE
        valid_position = (
            float(self.p('min_grasp_position')) <= position <=
            float(self.p('max_grasp_position')) if picking else
            position <= float(self.p('maximum_staging_closed_position')))
        timeout = float(self.p(
            'grasp_watchdog_timeout' if picking else 'empty_close_watchdog_timeout'))
        stable_time = float(self.p(
            'grasp_watchdog_stable_time' if picking else
            'empty_close_watchdog_stable_time'))
        if (elapsed < timeout or stable < stable_time or
                not valid_position or self.gripper_goal_handle is None):
            return
        self.grasp_watchdog_accept_cancel = True
        self.grasp_watchdog_cancel_requested = True
        description = 'grasp contact' if picking else 'empty close'
        self.report(
            f'close watchdog accepted stable {description} at {position:.4f}rad; '
            'cancelling non-terminating close action')
        self.gripper_goal_handle.cancel_goal_async()

    def update_open_watchdog(self):
        if (not self.pending_gripper or
                self.state != self.WAIT_PLACE_OPEN or
                self.open_watchdog_started is None or
                self.open_watchdog_cancel_requested):
            return
        position = self.positions.get(self.p('gripper_joint_name'))
        if position is None:
            return
        now = self.get_clock().now()
        epsilon = float(self.p('open_watchdog_position_epsilon'))
        if (self.open_watchdog_last_position is None or
                abs(position - self.open_watchdog_last_position) > epsilon):
            self.open_watchdog_last_position = position
            self.open_watchdog_stable_since = now
            return
        if self.open_watchdog_stable_since is None:
            self.open_watchdog_stable_since = now
            return
        elapsed = (now - self.open_watchdog_started).nanoseconds / 1e9
        stable = (now - self.open_watchdog_stable_since).nanoseconds / 1e9
        open_enough = (
            position >= float(self.p('minimum_open_position')) and
            abs(position - float(self.p('gripper_open_position'))) <=
            float(self.p('open_tolerance')))
        if (elapsed < float(self.p('open_watchdog_timeout')) or
                stable < float(self.p('open_watchdog_stable_time')) or
                not open_enough or self.gripper_goal_handle is None):
            return
        self.open_watchdog_accept_cancel = True
        self.open_watchdog_cancel_requested = True
        self.report(
            f'open watchdog accepted stable opening at {position:.4f}rad; '
            'cancelling non-terminating open action')
        self.gripper_goal_handle.cancel_goal_async()

    def on_gripper_result(self, opening, next_state, description, future):
        self.pending_gripper = False
        try:
            wrapped = future.result()
        except Exception as error:
            self.fail(f'{description} result error: {error}')
            return
        result = wrapped.result
        accepted_grasp_stall = (
            not opening and next_state == self.WAIT_PICK_CLOSE and
            wrapped.status == GoalStatus.STATUS_ABORTED and bool(result.stalled))
        accepted_watchdog_grasp = (
            not opening and next_state == self.WAIT_PICK_CLOSE and
            wrapped.status == GoalStatus.STATUS_CANCELED and
            self.grasp_watchdog_accept_cancel)
        accepted_watchdog_empty_close = (
            not opening and next_state in (
                self.WAIT_STAGING_CLOSE, self.WAIT_FINAL_CLOSE) and
            wrapped.status == GoalStatus.STATUS_CANCELED and
            self.grasp_watchdog_accept_cancel)
        accepted_watchdog_open = (
            opening and next_state == self.WAIT_PLACE_OPEN and
            wrapped.status == GoalStatus.STATUS_CANCELED and
            self.open_watchdog_accept_cancel)
        measured = self.positions.get(self.p('gripper_joint_name'))
        accepted_stalled_open = (
            opening and next_state == self.WAIT_PLACE_OPEN and
            wrapped.status == GoalStatus.STATUS_ABORTED and
            bool(result.stalled) and measured is not None and
            float(measured) >= float(self.p('minimum_open_position')) and
            abs(float(measured) - float(self.p('gripper_open_position'))) <=
            float(self.p('open_tolerance')))
        self.gripper_goal_handle = None
        self.grasp_watchdog_started = None
        self.grasp_watchdog_stable_since = None
        self.grasp_watchdog_last_position = None
        self.grasp_watchdog_cancel_requested = False
        self.grasp_watchdog_accept_cancel = False
        self.open_watchdog_started = None
        self.open_watchdog_stable_since = None
        self.open_watchdog_last_position = None
        self.open_watchdog_cancel_requested = False
        self.open_watchdog_accept_cancel = False
        if (wrapped.status != GoalStatus.STATUS_SUCCEEDED and
                not accepted_grasp_stall and not accepted_watchdog_grasp and
                not accepted_watchdog_empty_close and
                not accepted_watchdog_open and not accepted_stalled_open):
            self.fail(f'{description} action status={wrapped.status}')
            return
        position = self.positions.get(self.p('gripper_joint_name'))
        if position is None:
            self.fail(f'{description}: gripper feedback unavailable')
            return
        if opening:
            if position < float(self.p('minimum_open_position')):
                self.fail(f'{description}: insufficient opening ({position:.4f}rad)')
                return
            if accepted_watchdog_open:
                self.report(f'accepted watchdog open at {position:.4f}rad')
            if accepted_stalled_open:
                self.report(f'accepted stalled open at {position:.4f}rad')
            if next_state == self.WAIT_PLACE_OPEN:
                self.publish_place_target()
                ok, result = self.request(
                    'place_xy_transfer', self.WAIT_PLACE_RETRACT,
                    'place retract to high approach')
                if not ok:
                    self.fail(result)
            return
        if next_state == self.WAIT_PICK_CLOSE:
            if accepted_grasp_stall:
                self.report(f'accepted grasp contact stall at {position:.4f}rad')
            if accepted_watchdog_grasp:
                self.report(f'accepted watchdog grasp contact at {position:.4f}rad')
            if not (float(self.p('min_grasp_position')) <= position <=
                    float(self.p('max_grasp_position'))):
                self.fail(f'grasp position outside range ({position:.4f}rad)')
                return
            if bool(self.p('require_grasp_confirmation')):
                self.enter(self.WAIT_GRASP_CONFIRM,
                           f'gripper={position:.4f}rad; inspect grasp, then call ~/continue')
            else:
                ok, result = self.request('lift', self.WAIT_LIFT, 'loaded lift')
                if not ok:
                    self.fail(result)
            return
        if next_state == self.WAIT_STAGING_CLOSE:
            if accepted_watchdog_empty_close:
                self.report(f'accepted watchdog staging close at {position:.4f}rad')
            if position > float(self.p('maximum_staging_closed_position')):
                self.fail(f'staging gripper did not close ({position:.4f}rad)')
                return
            self.enter(
                self.WAIT_PICK_TARGET,
                f'staging complete; gripper={position:.4f}rad; select the box target')
            return
        if next_state == self.WAIT_FINAL_CLOSE:
            if accepted_watchdog_empty_close:
                self.report(f'accepted watchdog final close at {position:.4f}rad')
            self.enter(self.COMPLETE, f'cycle complete; final gripper={position:.4f}rad')

    def actual_tcp_z(self):
        names = list(self.p('joint_names'))
        if not all(name in self.positions for name in names):
            return None
        q1, q2, q3, q4, q5 = [self.positions[name] for name in names]

        def translation(x, y, z):
            matrix = np.eye(4)
            matrix[:3, 3] = [x, y, z]
            return matrix

        def rotation(axis, angle):
            cosine, sine = math.cos(angle), math.sin(angle)
            matrix = np.eye(4)
            if axis == 'z':
                matrix[:3, :3] = [
                    [cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]]
            elif axis == 'y':
                matrix[:3, :3] = [
                    [cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]]
            else:
                matrix[:3, :3] = [
                    [1, 0, 0], [0, cosine, -sine], [0, sine, cosine]]
            return matrix

        matrix = (
            translation(-0.01125, 0.0, 0.034) @ rotation('z', q1) @
            translation(0.0, 0.0, 0.0635) @ rotation('y', q2) @
            translation(0.0415, 0.0, 0.11315) @ rotation('y', q3) @
            translation(0.162, 0.0, 0.0) @ rotation('y', q4) @
            translation(0.0287, 0.0, 0.0) @ rotation('x', q5) @
            translation(0.09193, -0.0016, 0.0))
        return float(matrix[2, 3] + self.p('fk_position_bias')[2])

    def publish_place_target(self):
        xyz = list(self.p('remembered_place_xyz'))
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.p('base_frame')
        message.pose.position.x = float(xyz[0])
        message.pose.position.y = float(xyz[1])
        message.pose.position.z = float(xyz[2])
        message.pose.orientation.w = 1.0
        self.place_target_pub.publish(message)
        self.report(
            f'remembered place target published x={xyz[0]:.4f}, y={xyz[1]:.4f}')

    def publish_place_release_target(self):
        xyz = list(self.p('remembered_place_release_xyz'))
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.p('base_frame')
        message.pose.position.x = float(xyz[0])
        message.pose.position.y = float(xyz[1])
        message.pose.position.z = float(xyz[2])
        message.pose.orientation.w = 1.0
        self.place_release_target_pub.publish(message)
        self.report(
            f'remembered place release target published '
            f'x={xyz[0]:.4f}, y={xyz[1]:.4f}, z={xyz[2]:.4f}')

    def after_place_rotation(self):
        z = self.actual_tcp_z()
        if z is None:
            self.fail('cannot evaluate place correction: joint feedback unavailable')
            return
        if z < float(self.p('place_correction_trigger_z')):
            ok, result = self.request(
                'place_correction', self.WAIT_PLACE_CORRECTION,
                f'place lift correction (actual Z={z:.4f}m)')
        else:
            ok, result = self.request(
                'place_descent', self.WAIT_PLACE_DESCENT,
                f'place descent (actual Z={z:.4f}m)')
        if not ok:
            self.fail(result)


def main(args=None):
    rclpy.init(args=args)
    node = PickCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
