#!/usr/bin/env python3

from typing import Dict
import roslib
import sys
import rospy
import cv2
import numpy as np
import Process
import json
from Process import Image_processes
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64, UInt8MultiArray
from cv_bridge import CvBridge, CvBridgeError


class arm_mover1:

  # Defines publisher and subscriber
  def __init__(self):

    # initialize the node named image_processing
    rospy.init_node('arm_mover1', anonymous=True)
    # initialize a publisher to send xz coordinates
    self.joint4 = rospy.Publisher("/robot/joint4_position_controller/command", Float64 ,queue_size = 1)
    self.joint3 = rospy.Publisher("/robot/joint3_position_controller/command", Float64 ,queue_size = 1)
    self.joint2 = rospy.Publisher("/robot/joint2_position_controller/command", Float64 ,queue_size = 1)
    self.joint1 = rospy.Publisher("/robot/joint1_position_controller/command", Float64 ,queue_size = 1)
    self.bridge = CvBridge()
   
    rate = rospy.Rate(1)  # 5hz
    # record the beginning time
    stime = rospy.get_time()
    testcases = [
      [0,0,0,0],
      [0,-1.57,0,0], # Hide blue red camera 1
      [0,-1.5,1.5,1.5]
    ]
    i = 2
    while not rospy.is_shutdown():
      test = testcases[i]
      self.joint4.publish(test[3])
      self.joint3.publish(test[2])
      self.joint2.publish(test[1])
      self.joint1.publish(test[0])
      print(test)
      # input()
      # t = rospy.get_time()-stime
      # j2 = (np.pi/2)*np.sin((np.pi/15)*t)
      # j3 =(np.pi/2)*np.sin((np.pi/20)*t)
      # j4 =(np.pi/2)*np.sin((np.pi/18)*t)
      # self.joint4.publish(j4)
      # self.joint3.publish(j3)
      # self.joint2.publish(j2)
      #   #print("Sending")
      #   # self.showimg()
      # rate.sleep()


# call the class
def main(args):
  ic = arm_mover1()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


