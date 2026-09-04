#!/usr/bin/env python3
import time
import numpy as np
import rclpy, torch
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.act.modeling_act import ACTPolicy

NAMES=['joint1','joint2','joint3','joint4','joint5','gripper_joint_1']
class N(Node):
 def __init__(self):
  super().__init__('act_raw_publisher');self.b=CvBridge();self.im={};self.s=None;self.st={};self.steps=0
  self.pub=self.create_publisher(Float64MultiArray,'/act/raw_action',10)
  self.create_subscription(Image,'/wrist_camera/image_raw',lambda m:self.img('wrist_camera',m),1)
  self.create_subscription(Image,'/camera1/image_raw',lambda m:self.img('fixed_camera',m),1)
  self.create_subscription(JointState,'/joint_states',self.js,10)
  c=PreTrainedConfig.from_pretrained('baemseo/omx_box_v1');c.device='cpu';self.p=ACTPolicy.from_pretrained('baemseo/omx_box_v1',config=c,cache_dir='/tmp/hf_models');self.p.eval()
  self.create_timer(.15,self.tick);self.get_logger().info('ACT model ready, 55-step grasp-phase rollout')
 def img(self,k,m):self.im[k]=self.b.imgmsg_to_cv2(m,'rgb8');self.st[k]=time.monotonic()
 def js(self,m):
  d=dict(zip(m.name,m.position))
  if all(x in d for x in NAMES):self.s=np.array([d[x] for x in NAMES],np.float32);self.st['state']=time.monotonic()
 def tick(self):
  if self.steps>=55 or self.s is None or len(self.im)!=2:return
  now=time.monotonic()
  if any(now-self.st.get(k,0)>.5 for k in ['wrist_camera','fixed_camera','state']):return
  o={'observation.state':torch.from_numpy(self.s.copy()).unsqueeze(0)}
  for k,v in self.im.items():o['observation.images.'+k]=torch.from_numpy(v.copy()).permute(2,0,1).float().div(255).unsqueeze(0)
  with torch.inference_mode():a=self.p.select_action(o).squeeze().numpy()
  if a.size!=6 or not np.isfinite(a).all():self.get_logger().error('invalid action');return
  m=Float64MultiArray();m.data=a.astype(float).tolist();self.pub.publish(m);self.get_logger().info(f'{self.steps}: {np.round(a,4)}');self.steps+=1
  if self.steps>=55:self.get_logger().info('ACT grasp-phase rollout complete')
rclpy.init();n=N()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.shutdown()
