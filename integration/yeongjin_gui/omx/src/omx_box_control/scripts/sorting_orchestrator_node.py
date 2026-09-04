#!/usr/bin/env python3
"""Coordinate OMX placement and the post-place Beagle shuttle mission."""

import json
import math
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SortingOrchestrator(Node):
    PLACE_RESET_STATES = frozenset({
        'WAIT_PLACE_HIGH_XY_TRANSFER',
        'WAIT_BEAGLE_ARRIVAL',
        'WAIT_PLACE_XY_PITCH_APPROACH',
        'WAIT_PLACE_ROTATE',
        'WAIT_PLACE_LIFT_CORRECTION',
        'WAIT_PLACE_DESCENT',
        'WAIT_PLACE_GRIPPER_OPEN',
        'WAIT_PLACE_RETRACT',
    })

    def __init__(self):
        super().__init__('sorting_orchestrator')
        self.declare_parameter('console_command_topic', '/console/command')
        self.declare_parameter('selection_topic', '/console/selected_box')
        self.declare_parameter('status_topic', '/console/status')
        self.declare_parameter('beagle_command_topic', '/beagle/command')
        self.declare_parameter('beagle_status_topic', '/beagle/status')
        self.declare_parameter('job_event_topic', '/console/job_event')
        self.declare_parameter('coordinator_status_topic', '/pick_coordinator/status')
        self.declare_parameter('detections_topic', '/console/detections')
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
        self.declare_parameter('automatic_unload_omx', False)
        self.declare_parameter(
            'unload_omx_status_topic', '/unload_omx/unload_coordinator/status')
        self.declare_parameter('unload_vision_retry_interval', 0.5)
        self.declare_parameter('unload_vision_retry_timeout', 8.0)
        self.declare_parameter('continuous_operation', False)
        self.declare_parameter('auto_recover_failed_cycle', True)
        self.declare_parameter('failure_ready_delay', 1.0)
        # A clearly open gripper after closing is direct positive evidence that
        # a box is held. Thin boxes near the mechanical-close value still use
        # the camera check at the now-uncovered original location.
        self.declare_parameter('verification_gripper_joint_name', 'gripper_joint_1')
        self.declare_parameter('post_transfer_grasp_confirm_min_position', 0.15)
        self.declare_parameter('post_transfer_grasp_confirm_max_position', 0.70)
        self.declare_parameter('post_transfer_verify_radius_px', 25.0)
        self.declare_parameter('post_transfer_verify_min_frames', 4)
        self.declare_parameter('post_transfer_verify_match_frames', 2)
        self.declare_parameter('post_transfer_verify_timeout', 2.0)
        self.declare_parameter('home_joint_names',
                               ['joint1', 'joint2', 'joint3', 'joint4', 'joint5'])
        self.declare_parameter(
            'home_positions', [0.0, -1.57, 1.57, 1.57, 0.0])
        self.declare_parameter('home_move_duration', 5.0)
        self.declare_parameter('home_minimum_completion_time', 5.0)
        self.declare_parameter('home_joint_tolerance', 0.08)
        self.declare_parameter('home_settle_time', 0.20)
        self.declare_parameter('home_timeout', 12.0)
        self.declare_parameter('reset_home_retry_limit', 1)
        self.declare_parameter('gripper_action', '/gripper_controller/gripper_cmd')
        self.declare_parameter('reset_gripper_open_position', 0.98)
        self.declare_parameter('reset_gripper_closed_position', 0.0)
        self.declare_parameter('reset_gripper_effort', 10.0)
        self.declare_parameter('reset_gripper_min_open_position', 0.53)
        self.declare_parameter('reset_gripper_max_closed_position', 0.05)
        self.declare_parameter('reset_gripper_open_dwell', 2.0)
        self.declare_parameter('reset_gripper_timeout', 4.0)
        self.enabled = False
        self.estopped = False
        self.job = None
        # `job` is the box currently being delivered by Beagle.  Keep the
        # following detected defect separate so OMX can pick it while Beagle
        # returns with the previous box.
        self.next_job = None
        self.beagle_returning = False
        # Preserve the delivery identity after the Beagle leaves the unload
        # zone.  The delivery is logged as successful only when a later idle
        # status confirms that it has physically returned to the waiting zone.
        self.returning_delivery_job = None
        self.beagle_return_requested = False
        # Ignore the final WAIT_SIGNAL heartbeat that can race with a newly
        # queued box_placed command.  An idle status is a completed round trip
        # only after the Beagle has actually departed and return was requested.
        self.beagle_delivery_departed = False
        self.beagle_place_permitted = False
        self.place_release_requested = False
        self.last_robot_target = None
        self.joints = {}
        self.returning_home = False
        self.awaiting_operator_unload = False
        self.unload_omx_active = False
        self.unload_job = None
        self.unload_started_job_id = None
        self.unload_release_sent_job_id = None
        self.unload_omx_failure_logged = False
        self.unload_start_retry_started = None
        self.unload_start_retry_scheduled = None
        self.unload_start_retry_reason = None
        self.coordinator_state = 'IDLE'
        self.pick_continue_requested = False
        self.grasp_continue_requested = False
        self.parking = False
        self.home_goal = None
        self.home_started = None
        self.home_stable_since = None
        self.home_retry_count = 0
        self.home_completion_mode = None
        self.failure_recovery_active = False
        self.failure_ready_started = None
        self.failure_home_required = False
        self.pick_verify_active = False
        self.pick_verify_passed = False
        self.pick_verify_reject_requested = False
        self.pick_verify_started = None
        self.pick_verify_frames = 0
        self.pick_verify_match_frames = 0
        self.pick_verify_failure_reason = None
        self.reset_active = False
        self.reset_place_retract_required = False
        self.reset_place_retract_started = False
        self.reset_gripper_phase = None
        self.reset_gripper_pending = False
        self.reset_gripper_goal_handle = None
        self.reset_gripper_started = None
        self.reset_gripper_open_dwell_started = None
        self.reset_gripper_cancel_requested = False
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
        self.create_subscription(String, self.p('detections_topic'), self.on_detections, 10)
        self.create_subscription(
            String, self.p('unload_omx_status_topic'), self.on_unload_omx, 10)
        self.create_subscription(
            Float64MultiArray, self.p('external_yolo_topic'), self.on_external_yolo, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_timer(0.05, self.update_home)
        self.create_timer(0.05, self.update_reset_gripper)
        self.create_timer(0.05, self.update_failure_recovery)
        self.create_timer(0.05, self.update_pick_verification)
        self.create_timer(0.10, self.update_unload_start_retry)
        self.coordinator_start = self.create_client(Trigger, '/pick_coordinator/start')
        self.coordinator_continue = self.create_client(Trigger, '/pick_coordinator/continue')
        self.coordinator_cancel = self.create_client(Trigger, '/pick_coordinator/cancel')
        self.coordinator_release_place = self.create_client(
            Trigger, '/pick_coordinator/release_place')
        self.coordinator_reject_grasp = self.create_client(
            Trigger, '/pick_coordinator/reject_grasp')
        self.coordinator_reset_retract = self.create_client(
            Trigger, '/pick_coordinator/reset_retract')
        self.reset_gripper = ActionClient(
            self, GripperCommand, self.p('gripper_action'))
        self.unload_omx_start = self.create_client(
            Trigger, '/unload_omx/unload_coordinator/start')
        self.unload_omx_cancel = self.create_client(
            Trigger, '/unload_omx/unload_coordinator/cancel')
        self.unload_omx_reset = self.create_client(
            Trigger, '/unload_omx/unload_coordinator/reset')
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
        try:
            selection = json.loads(message.data)
            label = selection['class']
            if label != 'defect':
                raise ValueError('only defect boxes can be moved')
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.report(f'Selection rejected: {error}')
            return
        if self.beagle_returning or self.beagle_return_requested:
            self.queue_next_defect(selection)
        elif self.job or self.next_job:
            self.report('Selection ignored: a defect transfer is already in progress')
        else:
            self.start_defect_transfer(selection)

    def on_external_yolo(self, message):
        """Dispatch Beagle from the OMX integration interface before pick starts."""
        # The legacy Float64 bridge has no pixel or joint5 angle. Letting it
        # create a job races the GUI detector and leaves WAIT_PICK_TARGET with
        # no target that can be republished after staging.
        if not bool(self.get_parameter('accept_external_yolo').value):
            return
        if (not self.enabled or self.estopped or
                (self.job and not (self.beagle_returning or self.beagle_return_requested)) or
                self.failure_recovery_active or self.reset_active or
                self.parking or len(message.data) < 4):
            return
        is_defect, confidence, robot_x, robot_y = map(float, message.data[:4])
        if is_defect < 0.5 or confidence < float(self.p('external_yolo_minimum_confidence')):
            return
        if not all(math.isfinite(value) for value in (confidence, robot_x, robot_y)):
            return
        selection = {
            'class': 'defect', 'confidence': confidence,
            'robot_x': robot_x, 'robot_y': robot_y, 'source': 'omx_yolo_bridge'}
        if self.beagle_returning or self.beagle_return_requested:
            self.queue_next_defect(selection)
        else:
            self.start_defect_transfer(selection)

    def queue_next_defect(self, selection):
        """Keep only the newest stable defect while Beagle is returning."""
        if self.next_job is not None and self.next_job.get('job_id'):
            # Once staging starts, the retained pixel target and job identity
            # belong to the in-flight OMX cycle.  Replacing either while the
            # arm is moving can detach the final box_placed command from its
            # job and overwrite stage targets.
            self.report(
                'OMX_PICKING: parallel pick is active; new detection ignored')
            return
        job = dict(selection)
        if self.last_robot_target:
            job.update(self.last_robot_target)
        self.next_job = job
        if self.beagle_returning:
            self.report('BEAGLE_RETURNING_QUEUED: next defect waits for Beagle return')
        else:
            self.report('BEAGLE_RETURNING_QUEUED: next defect queued for Beagle return')

    def start_queued_pick(self):
        if (not self.beagle_returning or self.next_job is None or self.estopped or
                not self.enabled or self.failure_recovery_active or self.parking):
            return
        if self.coordinator_state not in ('IDLE', 'COMPLETE', 'FAILED'):
            return
        self.next_job['job_id'] = str(uuid.uuid4())
        self.beagle_place_permitted = False
        self.place_release_requested = False
        self.pick_continue_requested = False
        self.grasp_continue_requested = False
        self.reset_pick_verification()
        self.call(self.coordinator_start, 'OMX parallel staging')

    def start_defect_transfer(self, selection):
        if self._beagle_required() and not self._beagle_available():
            self.report(
                'Selection deferred: Beagle is not connected and ready at the receiving zone')
            return
        self.job = dict(selection)
        if self.last_robot_target:
            self.job.update(self.last_robot_target)
        self.job['job_id'] = str(uuid.uuid4())
        self.reset_pick_verification()
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
            if self.next_job:
                self.next_job.update(self.last_robot_target)
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
            self.beagle_place_permitted = False
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
            active_pick = self.next_job if self.beagle_returning else self.job
            if (active_pick is not None and not self.returning_home and
                    not self.awaiting_operator_unload):
                # Remember an early idle arrival until OMX reaches high place.
                self.beagle_place_permitted = True
        elif state in ('moving_to_defect', 'defect_arrived', 'returning'):
            self.beagle_connected = True
            self.beagle_ready = False

        incoming_job_id = status.get('job_id')
        if state in ('moving_to_defect', 'defect_arrived'):
            self.beagle_delivery_departed = True
        if state == 'defect_arrived' and self.job is None and incoming_job_id:
            # Recover from an earlier stale-idle race that detached the job
            # just before the real mission reported its defect-zone arrival.
            self.job = {'job_id': incoming_job_id}
            self.returning_home = True
            self.beagle_return_requested = False
            self.get_logger().warning(
                f'Restored Beagle delivery job {incoming_job_id} from defect arrival')

        if (state == 'idle' and self.returning_delivery_job is not None and
                incoming_job_id in (None, self.returning_delivery_job['job_id'])):
            completed_job = self.returning_delivery_job
            self.returning_delivery_job = None
            self.event_pub.publish(String(data=json.dumps({
                'event': 'return_completed',
                'job_id': completed_job['job_id'],
            })))
            # Keep return mode until a queued pick is started. The loading OMX
            # must remain stationary until Beagle has physically returned.
            if self.next_job is None:
                self.beagle_returning = False
            self.beagle_delivery_departed = False
            if self.next_job is not None:
                self.report(
                    'BEAGLE_HOME: returned to waiting zone; starting queued OMX pick')
                self.start_queued_pick()
            else:
                self.report('BEAGLE_HOME: returned to waiting zone; delivery complete')

        if not self.job or status.get('job_id') not in (None, self.job['job_id']):
            if state in ('failed', 'stopped'):
                # A return failure can refer to the completed delivery job,
                # which has deliberately been detached from `self.job` while
                # OMX works on `next_job`.  It still must block descent.
                self.beagle_failure_active = True
                self.beagle_place_permitted = False
                failure_key = (state, status.get('job_id'), status.get('detail'))
                if failure_key != self.last_beagle_failure_key:
                    self.last_beagle_failure_key = failure_key
                    self.publish_failure_event(
                        'beagle_operation_failed',
                        str(status.get('detail') or state), self.next_job)
                self.report(f'BEAGLE_{state.upper()}: OMX remains locked')
                return
            if state == 'idle' and self.next_job is not None:
                self.release_place_if_ready()
            if state == 'idle' and self.enabled and self.next_job is None:
                self.report('READY: Beagle waiting at receiving zone')
            return
        if state == 'signal_sent':
            detail = str(status.get('detail') or '')
            if 'operator_unloaded' in detail or 'box_picked' in detail:
                self.report('UNLOAD_SIGNAL_SENT: Beagle return command delivered')
            else:
                self.report('BEAGLE_SIGNAL_SENT: box placement handoff delivered')
        elif state == 'moving_to_defect':
            self.report('BEAGLE_DELIVERY: moving to defect zone')
        elif state == 'defect_arrived':
            if self.returning_home and not self.awaiting_operator_unload:
                self.awaiting_operator_unload = True
                self.unload_omx_failure_logged = False
                if bool(self.get_parameter('automatic_unload_omx').value):
                    self.start_unload_omx()
                elif bool(self.get_parameter('auto_complete_unload').value):
                    self.return_home_after_unload()
                else:
                    self.report(
                        'BEAGLE_DEFECT_ARRIVED: unload the box, then press 하역 완료')
        elif state == 'returning':
            # Detach the delivered job, but keep the loading OMX stationary
            # until Beagle reports idle at the physical receiving zone.
            completed_job = self.job
            self.returning_delivery_job = completed_job
            self.job = None
            self.returning_home = False
            self.awaiting_operator_unload = False
            self.beagle_returning = True
            self.beagle_return_requested = False
            self.beagle_delivery_departed = False
            self.beagle_place_permitted = False
            self.place_release_requested = False
            if self.next_job is not None:
                self.report('BEAGLE_RETURNING_QUEUED: next defect waits for Beagle return')
            else:
                self.report('BEAGLE_RETURNING: moving back; waiting for next defect')
        elif (state == 'idle' and self.returning_home and
              self.beagle_delivery_departed and self.beagle_return_requested):
            job_id = self.job['job_id']
            self.returning_home = False
            self.awaiting_operator_unload = False
            self.unload_omx_active = False
            self.beagle_delivery_departed = False
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
            self.beagle_place_permitted = False
            failure_key = (state, status.get('job_id'), status.get('detail'))
            if failure_key != self.last_beagle_failure_key:
                self.last_beagle_failure_key = failure_key
                self.publish_failure_event(
                    'beagle_operation_failed',
                    str(status.get('detail') or state), self.job)
            self.report(f'BEAGLE_{state.upper()}: OMX remains locked')

    def start_unload_omx(self):
        job_id = self.job.get('job_id') if self.job else None
        if self.unload_omx_active or (job_id and self.unload_started_job_id == job_id):
            return
        self.unload_started_job_id = job_id
        self.unload_job = dict(self.job) if self.job else None
        self.unload_start_retry_started = self.get_clock().now()
        self.unload_start_retry_scheduled = None
        self.unload_start_retry_reason = None
        self.request_unload_omx_start()

    def request_unload_omx_start(self):
        unload_job = self.unload_job
        if (not unload_job or self.unload_omx_active or not self.enabled or
                self.estopped or self.reset_active or
                not self.awaiting_operator_unload or self.beagle_return_requested):
            self.clear_unload_start_retry()
            return
        if not self.unload_omx_start.service_is_ready():
            self.handle_unload_omx_failure(
                'unload OMX start service is unavailable')
            return
        self.unload_omx_active = True
        self.report(
            'UNLOAD_OMX_STARTING: Beagle arrived; automatic unloading requested')
        future = self.unload_omx_start.call_async(Trigger.Request())
        future.add_done_callback(
            lambda result, job_id=unload_job['job_id']:
            self.on_unload_omx_start_done(result, job_id))

    def on_unload_omx_start_done(self, future, job_id):
        if not self.unload_job or self.unload_job.get('job_id') != job_id:
            return
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self.handle_unload_omx_failure(f'unload OMX start error: {error}')
            return
        if result is None or not result.success:
            reason = result.message if result is not None else 'empty start response'
            if self.schedule_unload_vision_retry(reason):
                return
            self.handle_unload_omx_failure(reason)
            return
        self.clear_unload_start_retry()

    @staticmethod
    def is_transient_unload_vision_failure(reason):
        reason = str(reason).lower()
        return any(text in reason for text in (
            'unload camera target is stale',
            'stable unload camera target is unavailable',
        ))

    def schedule_unload_vision_retry(self, reason):
        unload_job = self.unload_job
        if (not self.is_transient_unload_vision_failure(reason) or
                not unload_job or not self.awaiting_operator_unload or
                self.beagle_return_requested or self.estopped or
                self.reset_active or not self.enabled):
            return False
        self.unload_omx_active = False
        if self.unload_start_retry_started is None:
            self.unload_start_retry_started = self.get_clock().now()
        elapsed = (
            self.get_clock().now() - self.unload_start_retry_started
        ).nanoseconds / 1e9
        timeout = float(self.get_parameter('unload_vision_retry_timeout').value)
        if elapsed >= timeout:
            return False
        self.unload_start_retry_reason = str(reason)
        self.unload_start_retry_scheduled = self.get_clock().now()
        self.report(
            f'UNLOAD_VISION_WAIT: {reason}; waiting for a fresh landmark target '
            f'({max(0.0, timeout - elapsed):.1f}s remaining)')
        return True

    def update_unload_start_retry(self):
        if self.unload_start_retry_scheduled is None:
            return
        unload_job = self.unload_job
        if (not unload_job or not self.job or
                unload_job.get('job_id') != self.job.get('job_id') or
                not self.awaiting_operator_unload or self.beagle_return_requested or
                self.estopped or self.reset_active or not self.enabled):
            self.clear_unload_start_retry()
            return
        now = self.get_clock().now()
        elapsed = (now - self.unload_start_retry_started).nanoseconds / 1e9
        timeout = float(self.get_parameter('unload_vision_retry_timeout').value)
        if elapsed >= timeout:
            reason = self.unload_start_retry_reason or 'fresh unload landmark unavailable'
            self.clear_unload_start_retry()
            self.handle_unload_omx_failure(
                f'{reason}; automatic retry timed out after {timeout:.1f}s')
            return
        retry_elapsed = (
            now - self.unload_start_retry_scheduled).nanoseconds / 1e9
        if retry_elapsed < float(
                self.get_parameter('unload_vision_retry_interval').value):
            return
        self.unload_start_retry_scheduled = None
        self.request_unload_omx_start()

    def clear_unload_start_retry(self):
        self.unload_start_retry_started = None
        self.unload_start_retry_scheduled = None
        self.unload_start_retry_reason = None

    def on_unload_omx(self, message):
        unload_job = self.unload_job
        if not unload_job:
            return
        state, _, detail = message.data.partition(':')
        if (state == 'WAIT_ROTATE_TO_DESTINATION' and self.unload_omx_active and
                self.unload_release_sent_job_id != unload_job['job_id']):
            self.release_beagle_after_source_lift()
        elif (state == 'WAIT_ABORT_RETURN_STAGING' and self.unload_omx_active and
                self.unload_release_sent_job_id != unload_job['job_id']):
            # Retry exhaustion: the empty gripper is now safely above Beagle.
            self.release_beagle_after_source_lift()
        elif state == 'PICK_FAILED_COMPLETE' and self.unload_omx_active:
            self.unload_omx_active = False
            self.clear_unload_start_retry()
            if self.unload_release_sent_job_id != unload_job['job_id']:
                self.release_beagle_after_source_lift()
            if not self.unload_omx_failure_logged:
                self.unload_omx_failure_logged = True
                self.publish_failure_event(
                    'unload_pick_failed', detail.strip() or message.data, unload_job)
            self.report(
                'UNLOAD_PICK_FAILED: retries exhausted; unloading OMX parked and '
                'Beagle returning to receiving zone')
            self.unload_job = None
        elif state == 'COMPLETE' and self.unload_omx_active:
            self.unload_omx_active = False
            self.clear_unload_start_retry()
            if self.unload_release_sent_job_id != unload_job['job_id']:
                self.release_beagle_after_source_lift()
            self.report('UNLOAD_OMX_COMPLETE: box placed and unloading OMX parked')
            self.unload_job = None
        elif state == 'FAILED' and self.unload_omx_active:
            self.handle_unload_omx_failure(detail.strip() or message.data)
        elif self.unload_omx_active:
            self.report(f'UNLOAD_OMX_ACTIVE: {message.data}')

    def handle_unload_omx_failure(self, reason):
        self.unload_omx_active = False
        self.clear_unload_start_retry()
        unload_job = self.unload_job or self.job or self.returning_delivery_job
        if not self.unload_omx_failure_logged:
            self.unload_omx_failure_logged = True
            self.publish_failure_event('unload_omx_failed', reason, unload_job)
        released = bool(
            unload_job and self.unload_release_sent_job_id == unload_job.get('job_id'))
        if released:
            self.report(
                f'UNLOAD_OMX_FAILED_AFTER_RELEASE: {reason}; Beagle return '
                'continues and box_picked will not be sent again')
        else:
            self.report(
                f'UNLOAD_OMX_FAILED: {reason}; Beagle remains stopped. '
                'Inspect the unload OMX or unload manually, then press 하역 완료')
        self.unload_job = None

    def release_beagle_after_source_lift(self):
        delivery = self.unload_job or self.job
        if not delivery:
            return
        job_id = delivery['job_id']
        if self.unload_release_sent_job_id == job_id:
            return
        self.unload_release_sent_job_id = job_id
        self.clear_unload_start_retry()
        self.awaiting_operator_unload = False
        self.beagle_return_requested = True
        self.beagle_pub.publish(String(data=json.dumps({
            'command': 'box_picked', 'job_id': job_id})))
        self.report(
            'UNLOAD_SOURCE_LIFT_COMPLETE: box_picked sent; Beagle returning '
            'while unloading OMX finishes placement')

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
            self.pick_continue_requested = False
            self.grasp_continue_requested = False
            self.call(self.coordinator_start, 'OMX staging')

    def on_coordinator(self, message):
        state = message.data.split(':', 1)[0]
        self.coordinator_state = state
        if self.reset_active and self.reset_place_retract_started:
            if state == 'RESET_PLACE_RETRACT_COMPLETE':
                self.reset_place_retract_started = False
                self.reset_place_retract_required = False
                self.report(
                    'RESETTING: place clearance reached; returning OMX to folded HOME')
                if not self.start_home():
                    self.reset_active = False
                    self.report('RESET_FAILED: unable to start folded HOME return')
                return
            if state == 'FAILED':
                self.reset_place_retract_started = False
                self.reset_place_retract_required = False
                self.reset_active = False
                self.report(
                    f'RESET_FAILED: safe place retract failed: {message.data}; '
                    'OMX remains stopped')
                return
        active_pick = self.next_job if self.beagle_returning else self.job
        if not active_pick:
            return
        if state == 'WAIT_PICK_TARGET':
            detail = message.data.split(':', 1)[1] if ':' in message.data else ''
            # The first WAIT_PICK_TARGET means staging is complete but the
            # coordinator has cleared its old target.  Publish the retained
            # click and wait for its explicit "pick target received" status
            # before calling ~/continue; calling in the same callback races
            # the target subscriber and leaves the cycle stuck here.
            if 'pick target received' not in detail:
                self.pixel_pub.publish(String(data=json.dumps(active_pick)))
                self.report('TARGET_SYNC: restoring the selected box target')
                return
            if (bool(self.get_parameter('auto_continue_pick').value) and
                    not self.pick_continue_requested):
                self.pick_continue_requested = True
                self.report('TARGET_READY: auto-continue enabled; beginning pick flow')
                self.call(self.coordinator_continue, 'OMX continue')
            else:
                self.report('TARGET_READY: press 집기 계속 to begin the existing pick flow')
        elif state == 'WAIT_GRASP_CONFIRM':
            if (bool(self.get_parameter('auto_continue_pick').value) and
                    not self.grasp_continue_requested):
                self.grasp_continue_requested = True
                self.report('GRASP_READY: auto-continue enabled; beginning loaded lift')
                self.call(self.coordinator_continue, 'OMX loaded lift continue')
            else:
                self.report('GRASP_READY: inspect the grasp, then press 집기 계속')
        elif state == 'WAIT_BEAGLE_ARRIVAL':
            if not self.pick_verify_passed:
                self.start_pick_verification(active_pick)
            else:
                self.report('OMX_WAITING_FOR_BEAGLE: verified box is above receiving position')
                self.release_place_if_ready()
        elif state == 'COMPLETE' and not self.returning_home and not self.awaiting_operator_unload:
            # A parallel pick becomes Beagle's delivery job only after OMX has
            # completed the released place sequence.
            if self.beagle_returning:
                self.job = self.next_job
                self.next_job = None
                self.beagle_returning = False
                self.beagle_place_permitted = False
                self.place_release_requested = False
                active_pick = self.job
            if active_pick is None or not active_pick.get('job_id'):
                self.recover_failed_cycle(
                    'place handoff blocked: active job identity is missing')
                return
            self.event_pub.publish(String(data=json.dumps(
                dict(active_pick, event='awaiting_operator_unload'))))
            if bool(self.get_parameter('bypass_beagle').value):
                self.awaiting_operator_unload = True
                self.report('OMX_COMPLETE: press 하역 완료 after removing the box')
            else:
                self.returning_home = True
                self.beagle_delivery_departed = False
                self.beagle_return_requested = False
                self.beagle_pub.publish(String(data=json.dumps({
                    'command': 'box_placed', 'job_id': active_pick['job_id']})))
                self.report(
                    'OMX_COMPLETE: box placed; Beagle delivery started')
            if not self.start_home(completion_mode='loading_cycle'):
                self.report(
                    'LOADING_HOME_FAILED: box handoff completed, but OMX could not '
                    'start the camera-clear HOME return')
        elif state == 'FAILED':
            self.recover_failed_cycle(message.data)

    def release_place_if_ready(self):
        """Release one waiting place only when this Beagle is healthy and idle."""
        active_pick = self.next_job if self.beagle_returning else self.job
        if (self.coordinator_state != 'WAIT_BEAGLE_ARRIVAL' or
                active_pick is None or self.place_release_requested or
                not self.pick_verify_passed or self.pick_verify_reject_requested):
            return
        if (not self.beagle_place_permitted or not self.beagle_connected or
                self.beagle_failure_active or self.estopped):
            return
        if not self.coordinator_release_place.service_is_ready():
            self.report('OMX_WAITING_FOR_BEAGLE: place release service unavailable')
            return
        self.place_release_requested = True
        self.report('BEAGLE_READY_RELEASE_PLACE: Beagle idle; releasing OMX place descent')
        future = self.coordinator_release_place.call_async(Trigger.Request())
        future.add_done_callback(self.on_release_place_done)

    def on_release_place_done(self, future):
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self.place_release_requested = False
            self.report(f'OMX_WAITING_FOR_BEAGLE: place release error: {error}')
            return
        if result is None or not result.success:
            self.place_release_requested = False
            detail = result.message if result is not None else 'empty response'
            self.report(f'OMX_WAITING_FOR_BEAGLE: place release rejected: {detail}')

    def reset_pick_verification(self):
        self.pick_verify_active = False
        self.pick_verify_passed = False
        self.pick_verify_reject_requested = False
        self.pick_verify_started = None
        self.pick_verify_frames = 0
        self.pick_verify_match_frames = 0
        self.pick_verify_failure_reason = None

    def start_pick_verification(self, active_pick):
        if self.pick_verify_active or self.pick_verify_reject_requested:
            return
        gripper_position = self.joints.get(
            str(self.get_parameter('verification_gripper_joint_name').value))
        if (gripper_position is not None and
                float(self.get_parameter(
                    'post_transfer_grasp_confirm_min_position').value) <=
                float(gripper_position) <=
                float(self.get_parameter(
                    'post_transfer_grasp_confirm_max_position').value)):
            self.pick_verify_active = False
            self.pick_verify_passed = True
            self.report(
                f'PICK_VERIFIED: gripper contact confirms held box at '
                f'{float(gripper_position):.4f}rad')
            self.release_place_if_ready()
            return
        try:
            float(active_pick.get('center_x', active_pick['x']))
            float(active_pick.get('center_y', active_pick['y']))
        except (KeyError, TypeError, ValueError):
            self.reject_pick_after_transfer(
                'pick verification failed: original pixel target is unavailable')
            return
        self.pick_verify_active = True
        self.pick_verify_started = self.get_clock().now()
        self.pick_verify_frames = 0
        self.pick_verify_match_frames = 0
        self.report(
            'VERIFYING_PICK: checking that the defect left its original camera position')

    def on_detections(self, message):
        if not self.pick_verify_active or self.coordinator_state != 'WAIT_BEAGLE_ARRIVAL':
            return
        active_pick = self.next_job if self.beagle_returning else self.job
        if active_pick is None:
            return
        try:
            detections = json.loads(message.data).get('detections', [])
            target_x = float(active_pick.get('center_x', active_pick['x']))
            target_y = float(active_pick.get('center_y', active_pick['y']))
        except (AttributeError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        radius = max(
            1.0, float(self.get_parameter('post_transfer_verify_radius_px').value))
        target_still_present = any(
            detection.get('class') == 'defect' and
            math.hypot(
                float(detection.get('center_x', math.inf)) - target_x,
                float(detection.get('center_y', math.inf)) - target_y) <= radius
            for detection in detections
        )
        self.pick_verify_frames += 1
        if target_still_present:
            self.pick_verify_match_frames += 1
        else:
            self.pick_verify_match_frames = 0
        required_matches = max(
            1, int(self.get_parameter('post_transfer_verify_match_frames').value))
        required_frames = max(
            required_matches,
            int(self.get_parameter('post_transfer_verify_min_frames').value))
        if (self.pick_verify_frames >= required_frames and
                self.pick_verify_match_frames >= required_matches):
            self.reject_pick_after_transfer(
                'pick failed: defect remained at the original pick location')
            return
        if self.pick_verify_frames >= required_frames:
            self.pick_verify_active = False
            self.pick_verify_passed = True
            self.report(
                f'PICK_VERIFIED: original location clear across '
                f'{self.pick_verify_frames} detector frames')
            self.release_place_if_ready()

    def reject_pick_after_transfer(self, reason):
        if self.pick_verify_reject_requested:
            return
        self.pick_verify_active = False
        self.pick_verify_passed = False
        self.pick_verify_reject_requested = True
        self.pick_verify_failure_reason = reason
        self.beagle_place_permitted = False
        self.place_release_requested = False
        if not self.coordinator_reject_grasp.service_is_ready():
            self.pick_verify_reject_requested = False
            self.recover_failed_cycle(
                f'{reason}; post-transfer reject service unavailable')
            return
        self.report(f'PICK_REJECTED: {reason}; returning OMX to staging')
        future = self.coordinator_reject_grasp.call_async(Trigger.Request())
        future.add_done_callback(self.on_reject_grasp_done)

    def on_reject_grasp_done(self, future):
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self.pick_verify_reject_requested = False
            self.recover_failed_cycle(
                f'{self.pick_verify_failure_reason}; reject error: {error}')
            return
        if result is None or not result.success:
            self.pick_verify_reject_requested = False
            detail = result.message if result is not None else 'empty response'
            self.recover_failed_cycle(
                f'{self.pick_verify_failure_reason}; reject rejected: {detail}')

    def update_pick_verification(self):
        if not self.pick_verify_active or self.pick_verify_started is None:
            return
        elapsed = (
            self.get_clock().now() - self.pick_verify_started).nanoseconds / 1e9
        timeout = max(
            0.5, float(self.get_parameter('post_transfer_verify_timeout').value))
        if elapsed >= timeout:
            self.reject_pick_after_transfer(
                f'pick verification failed: only {self.pick_verify_frames} detector '
                f'frames received in {elapsed:.1f}s')

    def recover_failed_cycle(self, reason):
        """Cancel a failed cycle, discard its job, then reopen automatic detection."""
        if self.failure_recovery_active:
            return
        if not bool(self.get_parameter('auto_recover_failed_cycle').value):
            self.report('OMX_FAILED: inspect robot before reset')
            return

        failure_type = self.classify_omx_failure(reason)
        retry_pick = failure_type == 'box_pick_failed'
        failed_job = self.next_job if self.beagle_returning else self.job
        previous_delivery_in_transit = self.returning_delivery_job is not None

        self.failure_recovery_active = True
        self.failure_ready_started = None
        self.failure_home_required = (
            retry_pick and 'safely returned to staging' in str(reason).lower())
        self.awaiting_operator_unload = False
        self.unload_omx_active = False
        self.beagle_place_permitted = False
        self.place_release_requested = False
        self.reset_pick_verification()
        if retry_pick:
            # A pick failure occurs before Beagle delivery starts.  Keep the
            # Beagle stationary at the receiving zone and reopen detection
            # after the coordinator has safely returned to staging.
            if self.beagle_returning:
                self.next_job = None
                # If the previous delivery already reached idle, parallel
                # return mode is no longer needed for the replacement pick.
                if not previous_delivery_in_transit:
                    self.beagle_returning = False
            else:
                self.job = None
                self.next_job = None
                self.returning_home = False
                self.beagle_return_requested = False
                self.beagle_delivery_departed = False
            if not self.beagle_returning:
                self.beagle_ready = (
                    self.beagle_connected and not self.beagle_failure_active)
            self.publish_failure_event(failure_type, reason, failed_job)
            self.report(
                'RECOVERING: pick failed; returning to detection without stopping Beagle')
        else:
            # Placement or other robot failures may leave the load in an
            # unknown state, so preserve the existing fail-safe Beagle stop.
            self.returning_home = False
            self.beagle_returning = False
            self.returning_delivery_job = None
            self.beagle_return_requested = False
            self.beagle_delivery_departed = False
            self.job = None
            self.next_job = None
            self.publish_failure_event(failure_type, reason, failed_job)
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'stop',
                'job_id': failed_job.get('job_id') if failed_job else None,
            })))
            self.report('RECOVERING: OMX failure; current cycle cancelled')

        # Stop every stage as a defensive measure. The coordinator cancel response
        # is the synchronization point before detection is enabled again.
        for client in self.cancel_clients[1:]:
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        if self.unload_omx_cancel.service_is_ready():
            self.unload_omx_cancel.call_async(Trigger.Request())
        if self.coordinator_cancel.service_is_ready():
            future = self.coordinator_cancel.call_async(Trigger.Request())
            future.add_done_callback(self.on_failure_cancel_done)
        else:
            self.get_logger().warning(
                'Coordinator cancel service unavailable; FAILED is restartable')
            self.begin_failure_ready_delay()

    def reset_cycle(self):
        """Abort any stopped/incomplete cycle and restore the initial disabled state."""
        if self.reset_active:
            self.report('RESETTING: cycle cancellation is already active')
            return
        self.request_unload_reset()
        if self.parking:
            self.report(
                'RESETTING: loading OMX is already returning HOME; unloading OMX reset requested')
            return

        self.reset_place_retract_required = (
            self.reset_place_retract_required or
            self.coordinator_state in self.PLACE_RESET_STATES)
        self.reset_place_retract_started = False
        self.reset_active = True
        self.enabled = False
        self.returning_home = False
        self.beagle_returning = False
        self.returning_delivery_job = None
        self.beagle_return_requested = False
        self.beagle_delivery_departed = False
        self.beagle_place_permitted = False
        self.place_release_requested = False
        self.awaiting_operator_unload = False
        self.unload_omx_active = False
        self.clear_unload_start_retry()
        self.failure_recovery_active = False
        self.failure_ready_started = None
        self.reset_pick_verification()
        self.failure_home_required = False
        self.home_completion_mode = None
        self.unload_job = None
        self.unload_started_job_id = None
        self.unload_release_sent_job_id = None
        reset_job = self.job
        self.job = None
        self.next_job = None
        if reset_job is not None:
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'stop', 'job_id': reset_job.get('job_id')})))
        if not bool(self.get_parameter('bypass_beagle').value):
            # Reset the Beagle emergency latch immediately.  The Beagle
            # mission resets to stationary WAIT_SIGNAL, so this does not start
            # autonomous motion while OMX is returning HOME.
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'reconnect', 'job_id': None})))
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

    def request_unload_reset(self):
        if self.unload_omx_reset.service_is_ready():
            future = self.unload_omx_reset.call_async(Trigger.Request())
            future.add_done_callback(self.on_unload_reset_done)
        elif self.unload_omx_cancel.service_is_ready():
            self.unload_omx_cancel.call_async(Trigger.Request())
            self.report(
                'UNLOAD_RESET_WARNING: reset service unavailable; unload OMX held in place')
        else:
            self.report('UNLOAD_RESET_WARNING: unloading OMX is unavailable')

    def on_unload_reset_done(self, future):
        try:
            result = future.result()
            detail = result.message if result is not None else 'empty response'
            self.report(f'UNLOAD_RESET: {detail}')
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self.report(f'UNLOAD_RESET_FAILED: {error}')

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
        self.coordinator_state = 'IDLE'
        if self.reset_place_retract_required:
            if not self.coordinator_reset_retract.service_is_ready():
                self.reset_active = False
                self.report(
                    'RESET_FAILED: safe place retract service unavailable; '
                    'OMX remains stopped')
                return
            self.report('RESETTING: raising clear of the Beagle box before HOME return')
            future = self.coordinator_reset_retract.call_async(Trigger.Request())
            future.add_done_callback(self.on_reset_retract_requested)
            return
        self.report('RESETTING: returning OMX to folded HOME before releasing the box')
        if not self.start_home():
            self.reset_active = False
            self.report('RESET_FAILED: unable to start folded HOME return')

    def on_reset_retract_requested(self, future):
        if not self.reset_active:
            return
        try:
            result = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self.reset_place_retract_required = False
            self.reset_active = False
            self.report(
                f'RESET_FAILED: safe place retract request error: {error}; '
                'OMX remains stopped')
            return
        if result is None or not result.success:
            detail = result.message if result is not None else 'empty response'
            self.reset_place_retract_required = False
            self.reset_active = False
            self.report(
                f'RESET_FAILED: safe place retract rejected: {detail}; '
                'OMX remains stopped')
            return
        self.reset_place_retract_started = True
        self.report('RESETTING: waiting for measured high-place clearance')

    def complete_reset(self):
        """Finish reset only after HOME and the open/close release cycle."""
        self.reset_active = False
        self.estopped = False
        self.reset_place_retract_required = False
        self.reset_place_retract_started = False
        self.reset_gripper_phase = None
        self.reset_gripper_pending = False
        self.reset_gripper_open_dwell_started = None
        if self._beagle_required() and not self._beagle_available():
            self.report(
                'RESET: initial state restored; Beagle connection is in progress. Press 가동 after connected')
        else:
            self.report('RESET: HOME reached and gripper release cycle complete. Press 가동')

    def start_reset_gripper_cycle(self):
        if not self.reset_active:
            return
        if not self.reset_gripper.server_is_ready():
            self.report('RESET_FAILED: HOME reached but gripper action is unavailable')
            self.reset_active = False
            return
        self.reset_gripper_open_dwell_started = None
        self.reset_gripper_phase = 'OPEN'
        self.send_reset_gripper_goal(
            float(self.get_parameter('reset_gripper_open_position').value))

    def begin_reset_gripper_open_dwell(self, position):
        dwell = max(
            0.0, float(self.get_parameter('reset_gripper_open_dwell').value))
        self.reset_gripper_phase = 'OPEN_DWELL'
        self.reset_gripper_open_dwell_started = self.get_clock().now()
        self.report(
            f'RESETTING: gripper open at {position:.4f}rad; holding {dwell:.1f}s')

    def send_reset_gripper_goal(self, position):
        self.reset_gripper_pending = True
        self.reset_gripper_goal_handle = None
        self.reset_gripper_started = self.get_clock().now()
        self.reset_gripper_cancel_requested = False
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = float(
            self.get_parameter('reset_gripper_effort').value)
        phase = self.reset_gripper_phase
        future = self.reset_gripper.send_goal_async(goal)
        future.add_done_callback(
            lambda result: self.on_reset_gripper_goal(phase, result))
        self.report(f'RESETTING: gripper {phase.lower()} command requested')

    def on_reset_gripper_goal(self, phase, future):
        if not self.reset_active or phase != self.reset_gripper_phase:
            return
        try:
            handle = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self.reset_gripper_failed(phase, f'goal error: {error}')
            return
        if not handle.accepted:
            self.reset_gripper_failed(phase, 'goal rejected')
            return
        self.reset_gripper_goal_handle = handle
        handle.get_result_async().add_done_callback(
            lambda result: self.on_reset_gripper_result(phase, result))

    def on_reset_gripper_result(self, phase, future):
        if not self.reset_active or phase != self.reset_gripper_phase:
            return
        self.reset_gripper_pending = False
        self.reset_gripper_goal_handle = None
        self.reset_gripper_started = None
        self.reset_gripper_cancel_requested = False
        try:
            wrapped = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future boundary
            self.reset_gripper_failed(phase, f'result error: {error}')
            return
        measured = self.joints.get('gripper_joint_1')
        reached_position = (
            measured is not None and (
                measured >= float(self.get_parameter('reset_gripper_min_open_position').value)
                if phase == 'OPEN' else
                measured <= float(self.get_parameter('reset_gripper_max_closed_position').value)))
        if not reached_position:
            self.reset_gripper_failed(
                phase, f'action status={wrapped.status}; gripper feedback did not reach target')
            return
        if phase == 'OPEN':
            self.begin_reset_gripper_open_dwell(measured)
        else:
            self.complete_reset()

    def reset_gripper_failed(self, phase, reason):
        if self.reset_gripper_goal_handle is not None:
            self.reset_gripper_goal_handle.cancel_goal_async()
        self.reset_gripper_pending = False
        self.reset_gripper_phase = None
        self.reset_gripper_goal_handle = None
        self.reset_gripper_started = None
        self.reset_gripper_open_dwell_started = None
        self.reset_gripper_cancel_requested = False
        self.reset_active = False
        self.report(f'RESET_FAILED: gripper {phase.lower()} failed after HOME: {reason}')

    def complete_reached_reset_gripper_phase(self, position):
        """Advance from measured feedback without waiting for a stuck action result."""
        phase = self.reset_gripper_phase
        handle = self.reset_gripper_goal_handle
        self.reset_gripper_pending = False
        self.reset_gripper_goal_handle = None
        self.reset_gripper_started = None
        self.reset_gripper_cancel_requested = False
        self.report(
            f"RESETTING: gripper {phase.lower()} reached "
            f"{position:.4f}rad; advancing reset")
        if handle is not None:
            handle.cancel_goal_async()
        if phase == "OPEN":
            self.begin_reset_gripper_open_dwell(position)
        else:
            self.complete_reset()

    def update_reset_gripper(self):
        """Hold a reached opening, then accept feedback if ID16 hangs."""
        if not self.reset_active:
            return
        if self.reset_gripper_phase == 'OPEN_DWELL':
            if self.reset_gripper_open_dwell_started is None:
                return
            elapsed = (
                self.get_clock().now() -
                self.reset_gripper_open_dwell_started).nanoseconds / 1e9
            dwell = float(self.get_parameter('reset_gripper_open_dwell').value)
            if elapsed < max(0.0, dwell):
                return
            self.reset_gripper_open_dwell_started = None
            self.reset_gripper_phase = 'CLOSE'
            self.send_reset_gripper_goal(
                float(self.get_parameter('reset_gripper_closed_position').value))
            return
        if (not self.reset_gripper_pending or
                self.reset_gripper_started is None):
            return
        if self.reset_gripper_goal_handle is None or self.reset_gripper_cancel_requested:
            return
        position = self.joints.get("gripper_joint_1")
        reached = position is not None and (
            position >= float(self.get_parameter("reset_gripper_min_open_position").value)
            if self.reset_gripper_phase == "OPEN" else
            position <= float(self.get_parameter("reset_gripper_max_closed_position").value))
        if reached:
            self.complete_reached_reset_gripper_phase(position)
            return
        elapsed = (self.get_clock().now() - self.reset_gripper_started).nanoseconds / 1e9
        if elapsed >= float(self.get_parameter("reset_gripper_timeout").value):
            self.reset_gripper_failed(
                self.reset_gripper_phase, "timed out waiting for gripper feedback")

    @staticmethod
    def classify_omx_failure(reason):
        text = str(reason).lower()
        if any(token in text for token in ('place', 'unload', 'release')):
            return 'box_place_failed'
        if any(token in text for token in (
                'pick', 'grasp', 'gripper', 'empty grasp', 'wait_grasp',
                'staging', 'approach', 'pitch', 'xy')):
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
        if self.failure_home_required:
            self.failure_home_required = False
            if self.start_home(completion_mode='failure_recovery'):
                self.report(
                    'RECOVERING: safe staging reached; returning OMX to camera-clear HOME')
                return
            self.report('RECOVERING: unable to start camera-clear HOME return')
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
        elif self.beagle_returning:
            self.report(
                'BEAGLE_RETURNING: pick failure cleared; waiting for the next defect box')
        elif self._beagle_required() and not self._beagle_available():
            self.report(
                'WAIT_BEAGLE: pick failure cleared; waiting for Beagle at receiving zone')
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
        elif self.unload_omx_active:
            self.report(
                'Operator unload signal ignored: automatic unload OMX is still active')
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
            self.clear_unload_start_retry()
            self.awaiting_operator_unload = False
            self.beagle_return_requested = True
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'box_picked',
                'job_id': self.job['job_id']})))
            self.unload_release_sent_job_id = self.job['job_id']
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

        self.reset_place_retract_required = (
            self.coordinator_state in self.PLACE_RESET_STATES)
        self.reset_place_retract_started = False
        self.parking = False
        self.enabled = False
        self.estopped = True
        self.awaiting_operator_unload = False
        self.unload_omx_active = False
        self.clear_unload_start_retry()
        self.unload_job = None
        self.returning_home = False
        self.beagle_returning = False
        self.returning_delivery_job = None
        self.beagle_return_requested = False
        self.beagle_delivery_departed = False
        self.beagle_place_permitted = False
        self.place_release_requested = False
        self.next_job = None
        self.failure_recovery_active = False
        self.failure_ready_started = None
        self.failure_home_required = False
        self.home_completion_mode = None
        self.reset_active = False
        self.beagle_pub.publish(String(data=json.dumps({
            'command': 'stop', 'job_id': self.job.get('job_id') if self.job else None})))
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(Trigger.Request())
        if self.unload_omx_cancel.service_is_ready():
            self.unload_omx_cancel.call_async(Trigger.Request())
        names = [f'joint{i}' for i in range(1, 6)]
        if all(name in self.joints for name in names):
            hold = JointTrajectory(joint_names=names)
            point = JointTrajectoryPoint(positions=[self.joints[name] for name in names])
            point.time_from_start.nanosec = 100000000
            hold.points = [point]
            self.hold_pub.publish(hold)
        self.report('SOFTWARE_EMERGENCY_STOP: commands cancelled and hold requested; '
                    'use physical E-stop for immediate hardware stop')

    def start_home(self, retry=False, completion_mode=None):
        names = list(self.get_parameter('home_joint_names').value)
        positions = [float(value) for value in self.get_parameter('home_positions').value]
        if len(names) != len(positions) or not names:
            self.report('STOP_FAILED: invalid folded HOME joint configuration')
            return False
        if not all(name in self.joints for name in names):
            self.report('STOP_FAILED: joint feedback unavailable for folded HOME return')
            return False
        if not all(math.isfinite(value) for value in positions):
            self.report('STOP_FAILED: folded HOME positions are not finite')
            return False
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
        if not retry:
            self.home_retry_count = 0
            self.home_completion_mode = completion_mode
        self.parking = True
        if retry:
            prefix = 'RESETTING' if self.reset_active else 'LOADING_HOME'
            self.report(
                f'{prefix}: folded HOME trim retry {self.home_retry_count}/'
                f'{int(self.get_parameter("reset_home_retry_limit").value)}')
        else:
            self.report(
                'PARKING: returning OMX to folded bringup HOME '
                '[0, -1.57, 1.57, 1.57, 0]')
        return True

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
        if elapsed > float(self.get_parameter("home_timeout").value):
            retry_limit = int(self.get_parameter("reset_home_retry_limit").value)
            completion_mode = self.home_completion_mode
            retryable = (self.reset_active or
                         completion_mode in ('loading_cycle', 'failure_recovery'))
            if retryable and self.home_retry_count < retry_limit:
                self.home_retry_count += 1
                prefix = 'RESETTING' if self.reset_active else 'LOADING_HOME'
                self.report(
                    f'{prefix}: folded HOME near-target timeout; maximum joint '
                    f'error={math.degrees(maximum):.2f}deg; trim retry '
                    f'{self.home_retry_count}/{retry_limit}')
                if not self.start_home(retry=True):
                    self.parking = False
                    self.reset_active = False
                    self.report('RESET_FAILED: unable to start folded HOME trim retry')
                return
            self.parking = False
            completion_mode = self.home_completion_mode
            self.home_completion_mode = None
            if self.reset_active:
                self.reset_active = False
                self.report(
                    f'RESET_FAILED: folded HOME timeout; maximum joint error='
                    f'{math.degrees(maximum):.2f}deg; press 재설정 to retry')
            elif completion_mode in ('loading_cycle', 'failure_recovery'):
                self.enabled = False
                self.failure_recovery_active = False
                self.report(
                    f'LOADING_HOME_FAILED: folded HOME timeout; maximum joint error='
                    f'{math.degrees(maximum):.2f}deg; press 재설정 before continuing')
            else:
                self.report(
                    f'STOP_FAILED: folded HOME timeout; maximum joint error='
                    f'{math.degrees(maximum):.2f}deg')
            return
        if (maximum > float(self.get_parameter("home_joint_tolerance").value) or
                elapsed < float(self.get_parameter("home_minimum_completion_time").value)):
            self.home_stable_since = None
            return
        if self.home_stable_since is None:
            self.home_stable_since = now
            return
        if ((now - self.home_stable_since).nanoseconds / 1e9 >=
                float(self.get_parameter("home_settle_time").value)):
            self.parking = False
            self.home_retry_count = 0
            completion_mode = self.home_completion_mode
            self.home_completion_mode = None
            if self.reset_active:
                self.report(
                    f'RESETTING: folded HOME reached; maximum joint error='
                    f'{math.degrees(maximum):.2f}deg; opening gripper')
                self.start_reset_gripper_cycle()
            elif completion_mode == 'failure_recovery':
                self.report(
                    f'LOADING_HOME: folded HOME reached; maximum joint error='
                    f'{math.degrees(maximum):.2f}deg; camera view clear')
                self.begin_failure_ready_delay()
            elif completion_mode == 'loading_cycle':
                self.report(
                    f'LOADING_HOME: folded HOME reached; maximum joint error='
                    f'{math.degrees(maximum):.2f}deg; camera view clear')
            else:
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
