#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from tf import TransformBroadcaster
from sensor_msgs.msg import JointState
import rospy
import math
from rospy import Time 

# print(data.position)
a1 = math.radians(0)
a2 = math.radians(0)
a3 = math.radians(0)
print([a1,a2,a3])
#Rot Z
frame1 = np.array(
[[math.cos(a1),-math.sin(a1),0,4],
[math.sin(a1),math.cos(a1),0,0],
[0,0,1,0],
[0,0,0,1]])
#Rot x
frame2 = np.array(
[[1,0,0,3.2],
[0,math.cos(a2),-math.sin(a2),0],
[0,math.sin(a2),math.cos(a2),0],
[0,0,0,1]])
#Rot y
frame3 = np.array(
[[math.cos(a3),0,math.sin(a3),2.8],
[0,1,0,0],
[-math.sin(a3),0,math.cos(a3),0],
[0,0,0,1]])
end = frame3@frame2@frame1
translation = (end[0][3], end[1][3], end[2][3])
print(translation)
print(frame1)
print(frame2)
print(frame3)
print('-----------')
print(frame3@frame2@frame1)