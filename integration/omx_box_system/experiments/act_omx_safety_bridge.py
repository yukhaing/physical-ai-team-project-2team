#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from tf2_ros import Buffer,TransformListener
NAMES=['joint1','joint2','joint3','joint4','joint5','gripper_joint_1']
class N(Node):
 def __init__(self):
  super().__init__('act_omx_grasp_guard');self.s=None;self.last=None;self.arm_blocked=False;self.close_sent=False
  self.pub=self.create_publisher(JointTrajectory,'/arm_controller/joint_trajectory',10);self.g=ActionClient(self,GripperCommand,'/gripper_controller/gripper_cmd')
  self.tf=Buffer();self.tfl=TransformListener(self.tf,self)
  self.create_subscription(JointState,'/joint_states',self.js,10);self.create_subscription(Float64MultiArray,'/act/raw_action',self.cb,10);self.get_logger().warning('ACT grasp guard ARMED: floor Z=0.030m')
 def js(self,m):
  d=dict(zip(m.name,m.position))
  if all(x in d for x in NAMES):self.s=np.array([d[x] for x in NAMES],float)
 def z(self):
  try:return self.tf.lookup_transform('link0','end_effector_link',rclpy.time.Time()).transform.translation.z
  except Exception:return None
 def grip(self,pos,effort=8.):
  if self.g.server_is_ready():
   goal=GripperCommand.Goal();goal.command.position=float(pos);goal.command.max_effort=float(effort);self.g.send_goal_async(goal)
 def cb(self,m):
  if self.s is None:return
  a=np.array(m.data,float)
  if a.size!=6 or not np.isfinite(a).all():return
  z=self.z()
  if z is None:self.get_logger().error('STOP: no TF');self.arm_blocked=True
  if z is not None and z<=.030:
   if not self.arm_blocked:self.get_logger().warning(f'ARM FLOOR GUARD at Z={z:.4f}')
   self.arm_blocked=True
  if self.last is not None and abs(self.s[1]-self.last[1])>.15:
   self.get_logger().error('STOP: joint2 tracking error');self.arm_blocked=True
  if not self.arm_blocked:
   q=self.s.copy();q[:5]+=np.clip(a[:5]-self.s[:5],-.015,.015)
   t=JointTrajectory();t.joint_names=NAMES[:5];p=JointTrajectoryPoint();p.positions=q[:5].tolist();p.time_from_start=Duration(sec=0,nanosec=180000000);t.points=[p];self.pub.publish(t);self.last=q
  # Dataset-wide leader->follower regression: follower=0.5153469*raw+0.3427791.
  mapped=float(np.clip(.5153469*a[5]+.3427791,.30,.724))
  if a[5] < .30 and not self.close_sent:
   self.get_logger().warning(f'GRASP signal raw={a[5]:.3f}; command 0.340 effort 8')
   self.grip(.340,8.);self.close_sent=True;self.arm_blocked=True
  elif not self.close_sent:
   self.grip(mapped,5.)
rclpy.init();n=N()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.shutdown()
