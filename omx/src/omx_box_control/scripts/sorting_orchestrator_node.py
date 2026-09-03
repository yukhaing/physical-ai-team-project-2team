#!/usr/bin/env python3
"""Coordinate OMX placement and the post-place Beagle shuttle mission."""

import json
import math
import uuid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SortingOrchestrator(Node):
    def __init__(self):
        super().__init__('sorting_orchestrator')
        self.declare_parameter('console_command_topic', '/console/command')
        self.declare_parameter('selection_topic', '/console/selected_box')
        self.declare_parameter('status_topic', '/console/status')
        self.declare_parameter('beagle_command_topic', '/beagle/command')
        self.declare_parameter('beagle_status_topic', '/beagle/status')
        self.declare_parameter('job_event_topic', '/console/job_event')
        self.declare_parameter('coordinator_status_topic', '/pick_coordinator/status')
        self.declare_parameter('pixel_selection_topic', '/console/select_pixel')
        self.declare_parameter('movej_topic', '/omx_movej_controller/movej')
        self.declare_parameter('external_yolo_topic', '/yolo/selected_box')
        self.declare_parameter('accept_external_yolo', False)
        self.declare_parameter('external_yolo_minimum_confidence', 0.35)
        self.declare_parameter('bypass_beagle', False)
        self.declare_parameter('require_beagle_ready', True)
        self.declare_parameter('auto_start_omx', False)
        self.declare_parameter('auto_continue_pick', False)
        self.declare_parameter('auto_complete_unload', False)
        self.declare_parameter('continuous_operation', False)
        self.declare_parameter('auto_recover_failed_cycle', True)
        self.declare_parameter('failure_ready_delay', 1.0)
        self.declare_parameter('home_joint_names',
                               ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
        self.declare_parameter(
            'home_positions', [0.0, -1.57, 1.57, 1.57, 0.0])
        self.declare_parameter('home_move_duration', 5.0)
        self.declare_parameter('home_minimum_completion_time', 5.0)
        self.declare_parameter('home_joint_tolerance', 0.08)
        self.declare_parameter('home_settle_time', 0.20)
        self.declare_parameter('home_timeout', 12.0)
        self.enabled = False
        self.estopped = False
        self.job = None
        self.last_robot_target = None
        self.joints = {}
        self.returning_home = False
        self.awaiting_operator_unload = False
        self.coordinator_state = 'IDLE'
        self.parking = False
        self.home_goal = None
        self.home_started = None
        self.home_stable_since = None
        self.failure_recovery_active = False
        self.failure_ready_started = None
        self.reset_active = False
        self.beagle_connected = False
        self.beagle_ready = False
        self.beagle_disconnect_logged = False
        self.beagle_failure_active = False
        self.last_beagle_failure_key = None
        self.status_pub = self.create_publisher(String, self.p('status_topic'), 10)
        self.beagle_pub = self.create_publisher(String, self.p('beagle_command_topic'), 10)
        self.event_pub = self.create_publisher(String, self.p('job_event_topic'), 10)
        self.pixel_pub = self.create_publisher(String, self.p('pixel_selection_topic'), 10)
        self.hold_pub = self.create_publisher(JointTrajectory, self.p('movej_topic'), 10)
        self.create_subscription(String, self.p('console_command_topic'), self.on_command, 10)
        self.create_subscription(String, self.p('selection_topic'), self.on_selection, 10)
        self.create_subscription(String, '/console/robot_target', self.on_robot_target, 10)
        self.create_subscription(String, self.p('beagle_status_topic'), self.on_beagle, 10)
        self.create_subscription(String, self.p('coordinator_status_topic'), self.on_coordinator, 10)
        self.create_subscription(
            Float64MultiArray, self.p('external_yolo_topic'), self.on_external_yolo, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_timer(0.05, self.update_home)
        self.create_timer(0.05, self.update_failure_recovery)
        self.coordinator_start = self.create_client(Trigger, '/pick_coordinator/start')
        self.coordinator_continue = self.create_client(Trigger, '/pick_coordinator/continue')
        self.coordinator_cancel = self.create_client(Trigger, '/pick_coordinator/cancel')
        self.cancel_clients = [self.coordinator_cancel] + [
            self.create_client(Trigger, name) for name in (
            '/movej_staging/cancel', '/movej_xy_approach/cancel',
            '/movej_pitch_pregrasp/cancel', '/movej_lift/cancel',
            '/movej_place_xy_transfer/cancel', '/movej_place_recovery/cancel',
            '/gripper_open/cancel', '/movej_single_ptp_descent/cancel',
            '/movej_single_ptp_place_descent/cancel')]
        self.report('DISABLED: press 가동 to enable the system')

    def p(self, name):
        return str(self.get_parameter(name).value)

    def report(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def on_joints(self, message):
        self.joints.update(zip(message.name, message.position))

    def on_command(self, message):
        command = message.data.strip()
        self.get_logger().info(f'GUI command received: {command}')
        if command == 'enable':
            if self.parking:
                self.report('ENABLE_IGNORED: OMX is returning to folded HOME')
            elif self.reset_active:
                self.report('ENABLE_IGNORED: cycle reset is active')
            elif self.failure_recovery_active:
                self.report('ENABLE_IGNORED: failed cycle recovery is active')
            elif self.estopped:
                self.report('LOCKED: reset after emergency stop before enabling')
            else:
                self.enabled = True
                if self._beagle_required() and not self._beagle_available():
                    self.report(
                        'WAIT_BEAGLE: start the Beagle mission at the receiving zone')
                else:
                    self.report('READY: select a detected box')
        elif command == 'reset':
            self.reset_cycle()
        elif command == 'start_omx':
            self.start_omx()
        elif command == 'continue':
            self.call(self.coordinator_continue, 'OMX continue')
        elif command == 'operator_unloaded':
            self.return_home_after_unload()
        elif command in ('stop', 'estop'):
            self.stop(emergency=command == 'estop')

    def on_selection(self, message):
        if not self.enabled or self.estopped:
            self.report('Selection ignored: system is not enabled')
            return
        if self.failure_recovery_active or self.parking:
            self.report('Selection ignored: OMX is recovering')
            return
        if self.job:
            self.report('Selection ignored: a defect transfer is already in progress')
            return
        try:
            selection = json.loads(message.data)
            label = selection['class']
            if label != 'defect':
                raise ValueError('only defect boxes can be moved')
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.report(f'Selection rejected: {error}')
            return
        self.start_defect_transfer(selection)

    def on_external_yolo(self, message):
        """Dispatch Beagle from the OMX integration interface before pick starts."""
        # The legacy Float64 bridge has no pixel or joint5 angle. Letting it
        # create a job races the GUI detector and leaves WAIT_PICK_TARGET with
        # no target that can be republished after staging.
        if not bool(self.get_parameter('accept_external_yolo').value):
            return
        if (not self.enabled or self.estopped or self.job or
                self.failure_recovery_active or self.reset_active or
                self.parking or len(message.data) < 4):
            return
        is_defect, confidence, robot_x, robot_y = map(float, message.data[:4])
        if is_defect < 0.5 or confidence < float(self.p('external_yolo_minimum_confidence')):
            return
        if not all(math.isfinite(value) for value in (confidence, robot_x, robot_y)):
            return
        self.start_defect_transfer({
            'class': 'defect', 'confidence': confidence,
            'robot_x': robot_x, 'robot_y': robot_y, 'source': 'omx_yolo_bridge'})

    def start_defect_transfer(self, selection):
        if self._beagle_required() and not self._beagle_available():
            self.report(
                'Selection deferred: Beagle is not connected and ready at the receiving zone')
            return
        self.job = dict(selection)
        if self.last_robot_target:
            self.job.update(self.last_robot_target)
        self.job['job_id'] = str(uuid.uuid4())
        if bool(self.get_parameter('bypass_beagle').value):
            self.job['beagle_state'] = 'arrived'
            self.report('DEFECT_DETECTED: Beagle bypass enabled; OMX 집기 시작 가능')
            if bool(self.get_parameter('auto_start_omx').value):
                self.start_omx()
            return
        self.job['beagle_state'] = 'receiving'
        self.beagle_ready = False
        self.report('DEFECT_DETECTED: Beagle ready at receiving zone; OMX pick/place starting')
        if bool(self.get_parameter('auto_start_omx').value):
            self.start_omx()

    def on_robot_target(self, message):
        try:
            target = json.loads(message.data)
            self.last_robot_target = {
                'robot_x': float(target['robot_x']), 'robot_y': float(target['robot_y'])}
            if self.job:
                self.job.update(self.last_robot_target)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return

    def on_beagle(self, message):
        try:
            status = json.loads(message.data)
            state = status['state']
        except (KeyError, TypeError, json.JSONDecodeError):
            return
        if state == 'connected':
            self.beagle_connected = True
            self.beagle_disconnect_logged = False
            return
        if state == 'disconnected':
            was_connected = self.beagle_connected
            self.beagle_connected = False
            self.beagle_ready = False
            if (was_connected and not self.beagle_disconnect_logged and
                    not self.beagle_failure_active):
                self.beagle_disconnect_logged = True
                self.publish_failure_event(
                    'beagle_disconnected',
                    str(status.get('detail') or 'Beagle status connection lost'),
                    self.job)
            if self.enabled and not self.job:
                self.report('WAIT_BEAGLE: status connection lost')
            return
        if state == 'idle':
            self.beagle_connected = True
            self.beagle_ready = True
            self.beagle_disconnect_logged = False
            self.beagle_failure_active = False
            self.last_beagle_failure_key = None
        elif state in ('moving_to_defect', 'defect_arrived', 'returning'):
            self.beagle_connected = True
            self.beagle_ready = False

        if not self.job or status.get('job_id') not in (None, self.job['job_id']):
            if state == 'idle' and self.enabled:
                self.report('READY: Beagle waiting at receiving zone')
            return
        if state == 'signal_sent':
            detail = str(status.get('detail') or '')
            if 'operator_unloaded' in detail:
                self.report('UNLOAD_SIGNAL_SENT: Beagle return command delivered')
            else:
                self.report('BEAGLE_SIGNAL_SENT: box placement handoff delivered')
        elif state == 'moving_to_defect':
            self.report('BEAGLE_DELIVERY: moving to defect zone')
        elif state == 'defect_arrived':
            if self.returning_home and not self.awaiting_operator_unload:
                self.awaiting_operator_unload = True
                self.report(
                    'BEAGLE_DEFECT_ARRIVED: unload the box, then press 하역 완료')
                if bool(self.get_parameter('auto_complete_unload').value):
                    self.return_home_after_unload()
        elif state == 'returning':
            self.report('BEAGLE_RETURNING: moving back to receiving zone')
        elif state == 'idle' and self.returning_home:
            job_id = self.job['job_id']
            self.returning_home = False
            self.awaiting_operator_unload = False
            self.event_pub.publish(String(data=json.dumps({
                'event': 'return_completed', 'job_id': job_id})))
            self.job = None
            if (bool(self.get_parameter('continuous_operation').value) and
                    self.enabled and not self.estopped):
                self.report('READY: cycle complete; waiting for the next defect box')
            else:
                self.report('BEAGLE_HOME: defect transfer complete; ready for the next detection')
        elif state == 'stop_unsupported':
            self.report('BEAGLE_STOP_UNSUPPORTED: no remote stop was sent')
        elif state in ('failed', 'stopped'):
            self.beagle_failure_active = True
            failure_key = (state, status.get('job_id'), status.get('detail'))
            if failure_key != self.last_beagle_failure_key:
                self.last_beagle_failure_key = failure_key
                self.publish_failure_event(
                    'beagle_operation_failed',
                    str(status.get('detail') or state), self.job)
            self.report(f'BEAGLE_{state.upper()}: OMX remains locked')

    def publish_failure_event(self, failure_type, reason, job=None):
        event = dict(job or {})
        event.update({
            'event': 'cycle_failed',
            'job_id': event.get('job_id') or str(uuid.uuid4()),
            'failure_type': failure_type,
            'reason': reason,
        })
        self.event_pub.publish(String(data=json.dumps(event)))

    def _beagle_required(self):
        return (not bool(self.get_parameter('bypass_beagle').value) and
                bool(self.get_parameter('require_beagle_ready').value))

    def _beagle_available(self):
        return self.beagle_connected and self.beagle_ready

    def start_omx(self):
        if not self.enabled or self.estopped:
            self.report('OMX start blocked: system is disabled or emergency locked')
        elif not self.job or self.job.get('beagle_state') not in ('arrived', 'receiving'):
            self.report('OMX start blocked: wait for Beagle at receiving zone')
        else:
            self.call(self.coordinator_start, 'OMX staging')

    def on_coordinator(self, message):
        state = message.data.split(':', 1)[0]
        self.coordinator_state = state
        if not self.job:
            return
        if state == 'WAIT_PICK_TARGET':
            # The coordinator clears old targets at cycle start; restore the retained UI click.
            self.pixel_pub.publish(String(data=json.dumps(self.job)))
            if bool(self.get_parameter('auto_continue_pick').value):
                self.report('TARGET_READY: auto-continue enabled; beginning pick flow')
                self.call(self.coordinator_continue, 'OMX continue')
            else:
                self.report('TARGET_READY: press 집기 계속 to begin the existing pick flow')
        elif state == 'WAIT_GRASP_CONFIRM':
            if bool(self.get_parameter('auto_continue_pick').value):
                self.report('GRASP_READY: auto-continue enabled; beginning loaded lift')
                self.call(self.coordinator_continue, 'OMX loaded lift continue')
            else:
                self.report('GRASP_READY: inspect the grasp, then press 집기 계속')
        elif state == 'COMPLETE' and not self.returning_home and not self.awaiting_operator_unload:
            self.event_pub.publish(String(data=json.dumps(
                dict(self.job, event='awaiting_operator_unload'))))
            if bool(self.get_parameter('bypass_beagle').value):
                self.awaiting_operator_unload = True
                self.report('OMX_COMPLETE: press 하역 완료 after removing the box')
            else:
                self.returning_home = True
                self.beagle_pub.publish(String(data=json.dumps({
                    'command': 'box_placed', 'job_id': self.job['job_id']})))
                self.report(
                    'OMX_COMPLETE: box placed; Beagle delivery started')
        elif state == 'FAILED':
            self.recover_failed_cycle(message.data)

    def recover_failed_cycle(self, reason):
        """Cancel a failed cycle, discard its job, then reopen automatic detection."""
        if self.failure_recovery_active:
            return
        if not bool(self.get_parameter('auto_recover_failed_cycle').value):
            self.report('OMX_FAILED: inspect robot before reset')
            return

        self.failure_recovery_active = True
        self.failure_ready_started = None
        self.awaiting_operator_unload = False
        self.returning_home = False
        failed_job = self.job
        self.job = None
        self.publish_failure_event(
            self.classify_omx_failure(reason), reason, failed_job)
        self.beagle_pub.publish(String(data=json.dumps({
            'command': 'stop',
            'job_id': failed_job.get('job_id') if failed_job else None,
        })))
        self.report('RECOVERING: pick failed; current cycle cancelled')

        # Stop every stage as a defensive measure. The coordinator cancel response
        # is the synchronization point before detection is enabled again.
        for client in self.cancel_clients[1:]:
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        if self.coordinator_cancel.service_is_ready():
            future = self.coordinator_cancel.call_async(Trigger.Request())
            future.add_done_callback(self.on_failure_cancel_done)
        else:
            self.get_logger().warning(
                'Coordinator cancel service unavailable; FAILED is restartable')
            self.begin_failure_ready_delay()

    def reset_cycle(self):
        """Abort any stopped/incomplete cycle and restore the initial disabled state."""
        if self.parking:
            self.report('RESET_IGNORED: OMX is returning to folded HOME')
            return
        if self.reset_active:
            self.report('RESETTING: cycle cancellation is already active')
            return

        self.reset_active = True
        self.enabled = False
        self.estopped = False
        self.returning_home = False
        self.awaiting_operator_unload = False
        self.failure_recovery_active = False
        self.failure_ready_started = None
        reset_job = self.job
        self.job = None
        if reset_job is not None:
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'stop', 'job_id': reset_job.get('job_id')})))
        self.report('RESETTING: cancelling the current cycle')

        for client in self.cancel_clients[1:]:
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        if self.coordinator_cancel.service_is_ready():
            future = self.coordinator_cancel.call_async(Trigger.Request())
            future.add_done_callback(self.on_reset_cancel_done)
        else:
            self.get_logger().warning(
                'Coordinator cancel service unavailable during reset')
            self.finish_reset()

    def on_reset_cancel_done(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(f'Cycle reset coordinator: {result.message}')
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary
            self.get_logger().warning(f'Cycle reset coordinator call failed: {error}')
        self.finish_reset()

    def finish_reset(self):
        if not self.reset_active:
            return
        self.reset_active = False
        self.coordinator_state = 'IDLE'
        if self._beagle_required() and not self._beagle_available():
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'reconnect', 'job_id': None})))
            self.report(
                'RESET: initial state restored; Beagle reconnect requested. Press 가동')
        else:
            self.report('RESET: cycle cancelled; initial state restored. Press 가동')

    @staticmethod
    def classify_omx_failure(reason):
        text = str(reason).lower()
        if any(token in text for token in ('place', 'unload', 'release')):
            return 'box_place_failed'
        if any(token in text for token in (
                'pick', 'grasp', 'gripper', 'empty grasp', 'wait_grasp')):
            return 'box_pick_failed'
        return 'omx_operation_failed'

    def on_failure_cancel_done(self, future):
        try:
            result = future.result()
            if result is not None:
                self.get_logger().info(
                    f'Failed-cycle coordinator reset: {result.message}')
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary
            self.get_logger().warning(
                f'Failed-cycle coordinator reset call failed: {error}')
        self.begin_failure_ready_delay()

    def begin_failure_ready_delay(self):
        if not self.failure_recovery_active:
            return
        self.failure_ready_started = self.get_clock().now()
        delay = max(0.0, float(self.get_parameter('failure_ready_delay').value))
        self.report(f'RECOVERING: failed cycle cleared; READY in {delay:.1f}s')

    def update_failure_recovery(self):
        if not self.failure_recovery_active or self.failure_ready_started is None:
            return
        elapsed = (
            self.get_clock().now() - self.failure_ready_started).nanoseconds / 1e9
        if elapsed < max(0.0, float(self.get_parameter('failure_ready_delay').value)):
            return
        self.failure_recovery_active = False
        self.failure_ready_started = None
        if self.estopped:
            self.report('LOCKED: failed cycle cleared; reset emergency stop')
        elif self.enabled:
            self.report('READY: failed cycle cleared; waiting for the next defect box')
        else:
            self.report('DISABLED: failed cycle cleared; press 가동 to enable the system')

    def call(self, client, description):
        if not client.service_is_ready():
            self.report(f'{description} unavailable')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda result: self.report(
            f'{description}: {result.result().message}' if result.result() else f'{description} failed'))

    def return_home_after_unload(self):
        if not self.enabled or self.estopped:
            self.report('Return blocked: system is disabled or emergency locked')
        elif not self.job or not self.awaiting_operator_unload:
            self.report('Operator unload signal ignored: no defect box is waiting at the loading station')
        elif bool(self.get_parameter('bypass_beagle').value):
            job_id = self.job['job_id']
            self.awaiting_operator_unload = False
            self.event_pub.publish(String(data=json.dumps({
                'event': 'return_completed', 'job_id': job_id})))
            self.job = None
            if bool(self.get_parameter('continuous_operation').value):
                self.report('READY: cycle complete; continuous operation is waiting for the next defect box')
            else:
                self.report('BEAGLE_HOME: Beagle bypass enabled; defect transfer complete; ready for the next detection')
        else:
            self.awaiting_operator_unload = False
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'operator_unloaded',
                'job_id': self.job['job_id']})))
            self.report(
                'UNLOAD_COMPLETE: return signal sent; waiting for Beagle to return')

    def stop(self, emergency):
        if not emergency:
            if self.parking:
                self.report('PARKING: folded HOME return is already active')
                return
            cycle_active = (
                self.job is not None or self.returning_home or
                self.awaiting_operator_unload or
                self.failure_recovery_active or
                self.coordinator_state not in ('IDLE', 'COMPLETE'))
            if cycle_active:
                self.report(
                    f'STOP_IGNORED: cycle is active ({self.coordinator_state}); '
                    'press 정지 again only after the cycle completes')
                return
            self.enabled = False
            self.start_home()
            return

        self.parking = False
        self.enabled = False
        self.estopped = True
        self.awaiting_operator_unload = False
        self.returning_home = False
        self.failure_recovery_active = False
        self.failure_ready_started = None
        self.reset_active = False
        self.beagle_pub.publish(String(data=json.dumps({
            'command': 'stop', 'job_id': self.job.get('job_id') if self.job else None})))
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        names = [f'joint{i}' for i in range(1, 6)]
        if all(name in self.joints for name in names):
            hold = JointTrajectory(joint_names=names)
            point = JointTrajectoryPoint(positions=[self.joints[name] for name in names])
            point.time_from_start.nanosec = 100000000
            hold.points = [point]
            self.hold_pub.publish(hold)
        self.report('SOFTWARE_EMERGENCY_STOP: commands cancelled and hold requested; '
                    'use physical E-stop for immediate hardware stop')

    def start_home(self):
        names = list(self.get_parameter('home_joint_names').value)
        positions = [float(value) for value in self.get_parameter('home_positions').value]
        if len(names) != len(positions) or not names:
            self.report('STOP_FAILED: invalid folded HOME joint configuration')
            return
        if not all(name in self.joints for name in names):
            self.report('STOP_FAILED: joint feedback unavailable for folded HOME return')
            return
        if not all(math.isfinite(value) for value in positions):
            self.report('STOP_FAILED: folded HOME positions are not finite')
            return
        trajectory = JointTrajectory(joint_names=names)
        point = JointTrajectoryPoint(positions=positions)
        duration = max(0.02, float(self.get_parameter('home_move_duration').value))
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1.0e9)
        trajectory.points = [point]
        self.hold_pub.publish(trajectory)
        self.home_goal = dict(zip(names, positions))
        self.home_started = self.get_clock().now()
        self.home_stable_since = None
        self.parking = True
        self.report(
            'PARKING: returning OMX to folded bringup HOME '
            '[0, -1.57, 1.57, 1.57, 0]')

    def update_home(self):
        if not self.parking or self.home_goal is None:
            return
        now = self.get_clock().now()
        elapsed = (now - self.home_started).nanoseconds / 1e9
        errors = [
            abs(self.joints.get(name, math.inf) - goal)
            for name, goal in self.home_goal.items()
        ]
        maximum = max(errors)
        if elapsed > float(self.get_parameter('home_timeout').value):
            self.parking = False
            self.report(
                f'STOP_FAILED: folded HOME timeout; maximum joint error='
                f'{math.degrees(maximum):.2f}deg')
            return
        if (maximum > float(self.get_parameter('home_joint_tolerance').value) or
                elapsed < float(self.get_parameter('home_minimum_completion_time').value)):
            self.home_stable_since = None
            return
        if self.home_stable_since is None:
            self.home_stable_since = now
            return
        if ((now - self.home_stable_since).nanoseconds / 1e9 >=
                float(self.get_parameter('home_settle_time').value)):
            self.parking = False
            self.report(
                f'STOPPED: folded HOME reached; maximum joint error='
                f'{math.degrees(maximum):.2f}deg')


def main(args=None):
    rclpy.init(args=args)
    node = SortingOrchestrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
