#!/usr/bin/env python3
"""Gate OMX pick/place work on Beagle arrival and coordinate software stop requests."""

import json
import uuid

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
        self.enabled = False
        self.estopped = False
        self.job = None
        self.last_robot_target = None
        self.joints = {}
        self.returning_home = False
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
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.coordinator_start = self.create_client(Trigger, '/pick_coordinator/start')
        self.coordinator_continue = self.create_client(Trigger, '/pick_coordinator/continue')
        self.cancel_clients = [self.create_client(Trigger, name) for name in (
            '/pick_coordinator/cancel', '/movej_staging/cancel', '/movej_xy_approach/cancel',
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
        if command == 'enable':
            if self.estopped:
                self.report('LOCKED: reset after emergency stop before enabling')
            else:
                self.enabled = True
                self.report('READY: select a detected box')
        elif command == 'reset':
            self.estopped = False
            self.enabled = False
            self.job = None
            self.report('RESET: inspect hardware, then press 가동')
        elif command == 'start_omx':
            self.start_omx()
        elif command == 'continue':
            self.call(self.coordinator_continue, 'OMX continue')
        elif command in ('stop', 'estop'):
            self.stop(emergency=command == 'estop')

    def on_selection(self, message):
        if not self.enabled or self.estopped:
            self.report('Selection ignored: system is not enabled')
            return
        try:
            selection = json.loads(message.data)
            label = selection['class']
            if label not in ('normal', 'defect'):
                raise ValueError('class must be normal or defect')
        except (KeyError, ValueError, TypeError, json.JSONDecodeError) as error:
            self.report(f'Selection rejected: {error}')
            return
        self.job = dict(selection)
        if self.last_robot_target:
            self.job.update(self.last_robot_target)
        self.job['job_id'] = str(uuid.uuid4())
        self.job['beagle_state'] = 'moving'
        destination = f'{label}_loading'
        self.beagle_pub.publish(String(data=json.dumps({
            'command': destination, 'job_id': self.job['job_id']})))
        self.report(f'BEAGLE_MOVING: {destination}; OMX is locked until arrival')

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
        if not self.job or status.get('job_id') not in (None, self.job['job_id']):
            return
        if state == 'arrived' and not self.returning_home:
            self.job['beagle_state'] = 'arrived'
            self.report('BEAGLE_ARRIVED: OMX 집기 시작 가능')
        elif state == 'idle' and self.returning_home:
            self.returning_home = False
            self.event_pub.publish(String(data=json.dumps({
                'event': 'return_completed', 'job_id': self.job['job_id']})))
            self.report('BEAGLE_HOME: job complete')
        elif state in ('failed', 'stopped'):
            self.report(f'BEAGLE_{state.upper()}: OMX remains locked')

    def start_omx(self):
        if not self.enabled or self.estopped:
            self.report('OMX start blocked: system is disabled or emergency locked')
        elif not self.job or self.job.get('beagle_state') != 'arrived':
            self.report('OMX start blocked: wait for Beagle arrival')
        else:
            self.call(self.coordinator_start, 'OMX staging')

    def on_coordinator(self, message):
        state = message.data.split(':', 1)[0]
        if not self.job:
            return
        if state == 'WAIT_PICK_TARGET':
            # The coordinator clears old targets at cycle start; restore the retained UI click.
            self.pixel_pub.publish(String(data=json.dumps(self.job)))
            self.report('TARGET_READY: press 집기 계속 to begin the existing pick flow')
        elif state == 'COMPLETE' and not self.returning_home:
            self.event_pub.publish(String(data=json.dumps(dict(self.job, event='omx_completed'))))
            self.returning_home = True
            self.beagle_pub.publish(String(data=json.dumps({
                'command': 'home', 'job_id': self.job['job_id']})))
            self.report('OMX_COMPLETE: Beagle returning home')
        elif state == 'FAILED':
            self.report('OMX_FAILED: inspect robot before reset')

    def call(self, client, description):
        if not client.service_is_ready():
            self.report(f'{description} unavailable')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda result: self.report(
            f'{description}: {result.result().message}' if result.result() else f'{description} failed'))

    def stop(self, emergency):
        self.enabled = False
        self.estopped = self.estopped or emergency
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
        label = 'SOFTWARE_EMERGENCY_STOP' if emergency else 'STOPPED'
        self.report(f'{label}: commands cancelled and hold requested; use physical E-stop for immediate hardware stop')


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
