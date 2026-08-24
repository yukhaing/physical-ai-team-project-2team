#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from ultralytics import YOLO

CALIBRATION='/tmp/omx_camera_homography_7point.yaml'
MODEL='/tmp/box_defect_best.pt'

def load_calibration():
 fs=cv2.FileStorage(CALIBRATION,cv2.FILE_STORAGE_READ)
 h=fs.getNode('homography').mat() if fs.isOpened() else None
 image=fs.getNode('image_points').mat() if fs.isOpened() else None
 reference=fs.getNode('reference_points_link0').mat() if fs.isOpened() else None
 fs.release()
 if h is None or h.shape!=(3,3) or not np.isfinite(h).all():raise RuntimeError('invalid homography')
 image=image.reshape(-1,2).astype(float);reference=reference.reshape(-1,2).astype(float)
 projected=cv2.perspectiveTransform(image.reshape(-1,1,2),h).reshape(-1,2)
 return h,image,reference-projected

def n90(a):return (a+45)%90-45

class N(Node):
 def __init__(self):
  super().__init__('yolo_calibrated_detection_angle');self.h,self.cal_pixels,self.residuals=load_calibration();self.m=YOLO(MODEL);self.b=CvBridge()
  self.p1=self.create_publisher(Image,'/yolo/annotated',1);self.p2=self.create_publisher(Image,'/yolo/angle_annotated',1)
  self.target_pub=self.create_publisher(Float64MultiArray,'/yolo/selected_box',10)
  self.create_subscription(Image,'/camera1/image_raw',self.cb,1)
  self.get_logger().info('new 7-point homography active; normal>=0.75 defect>=0.35')
 def world(self,p):
  pixel=np.asarray(p,dtype=float);q=self.h@np.array([pixel[0],pixel[1],1.]);raw=q[:2]/q[2]
  distance=np.linalg.norm(self.cal_pixels-pixel,axis=1)
  nearest=int(np.argmin(distance))
  if distance[nearest]<1.0:return raw+self.residuals[nearest]
  weights=1.0/(distance*distance+1e-6);correction=(weights[:,None]*self.residuals).sum(axis=0)/weights.sum()
  return raw+correction
 def cb(self,msg):
  im=self.b.imgmsg_to_cv2(msg,'bgr8');det=im.copy();ang=im.copy()
  with torch.inference_mode():r=self.m.predict(im,conf=.35,verbose=False,device='cpu')[0]
  found=[]
  for b in r.boxes:
   name=self.m.names[int(b.cls.item())];cf=float(b.conf.item())
   if (name=='normal' and cf<.75) or (name=='defect' and cf<.35):continue
   x1,y1,x2,y2=map(int,b.xyxy[0]);u=(x1+x2)/2;v=(y1+y2)/2;X,Y=self.world((u,v));found.append((name,cf,X,Y,x1,y1,x2,y2,u,v))
  ds=[d for d in found if d[0]=='defect'];pool=ds or [d for d in found if d[0]=='normal'];sel=max(pool,key=lambda d:d[1]) if pool else None
  for d in found:
   name,cf,X,Y,x1,y1,x2,y2,u,v=d;col=(0,0,255) if name=='defect' else (0,200,0)
   cv2.rectangle(det,(x1,y1),(x2,y2),col,3 if d is sel else 2);cv2.circle(det,(int(u),int(v)),6,(255,255,0),-1)
   cv2.putText(det,f"{'SELECT ' if d is sel else ''}{name} {cf:.2f} X={X:.3f} Y={Y:.3f}",(max(5,min(det.shape[1]-420,x1)),max(24,y1-8)),cv2.FONT_HERSHEY_SIMPLEX,.6,col,2)
  selected_q5=float('nan')
  if ds:
   d=max(ds,key=lambda z:z[1]);_,cf,X,Y,x1,y1,x2,y2,u,v=d;xa=max(0,x1-8);ya=max(0,y1-8);xb=min(im.shape[1],x2+8);yb=min(im.shape[0],y2+8)
   hsv=cv2.cvtColor(im[ya:yb,xa:xb],cv2.COLOR_BGR2HSV);mask=((hsv[:,:,1]>18)&(hsv[:,:,1]<150)&(hsv[:,:,2]<245)&(hsv[:,:,0]>4)&(hsv[:,:,0]<35)).astype(np.uint8)*255
   mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,np.ones((9,9),np.uint8));mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5,5),np.uint8));cs,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE);cs=[c for c in cs if cv2.contourArea(c)>1000]
   if cs:
    box=cv2.boxPoints(cv2.minAreaRect(max(cs,key=cv2.contourArea)));box[:,0]+=xa;box[:,1]+=ya;edges=[(np.linalg.norm(box[(i+1)%4]-box[i]),box[i],box[(i+1)%4]) for i in range(4)];_,a,e=max(edges,key=lambda z:z[0]);wa,we=self.world(a),self.world(e);axis=n90(math.degrees(math.atan2((we-wa)[1],(we-wa)[0])));q1=math.degrees(math.atan2(Y,X+.01125));q5=n90(q1+90-axis+2.00255)
    selected_q5=math.radians(q5)
    cv2.polylines(ang,[box.astype(int)],True,(255,0,255),3);cv2.putText(ang,f'defect {cf:.2f} X={X:.3f} Y={Y:.3f} axis={axis:.1f} joint5={selected_q5:.3f}rad',(8,28),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,0,255),2)
  if sel is not None:
   name,cf,X,Y,*_=sel
   target=Float64MultiArray();target.data=[1.0 if name=='defect' else 0.0,cf,X,Y,selected_q5]
   self.target_pub.publish(target)
  for out,pub in ((det,self.p1),(ang,self.p2)):
   m=self.b.cv2_to_imgmsg(out,'bgr8');m.header=msg.header;pub.publish(m)

rclpy.init();n=N()
try:rclpy.spin(n)
except KeyboardInterrupt:pass
finally:n.destroy_node();rclpy.shutdown()
