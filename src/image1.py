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


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send xz coordinates
    self.imyz = rospy.Publisher("Coords", String ,queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()


  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      imageprocessor1 = Image_processes()
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    yzcontours = imageprocessor1.imProcess(self.cv_image1)
    yzCentres = imageprocessor1.Contours(yzcontours)

    # image_copy = self.cv_image1.copy()
    # for c in yzCentres:
    #     cv2.circle(image_copy, (c[0], c[1]), 2, (255, 255, 255), -1)
    #     cv2.putText(image_copy, "centroid", (c[0] - 25, c[1] - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # cv2.imshow("Centriods",image_copy)
    # cv2.waitKey(1)

    # Publish the results
    try: 
      self.imyz.publish(json.dumps(yzCentres))
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


