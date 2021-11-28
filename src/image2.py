#!/usr/bin/env python3


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
    self.imxz = rospy.Publisher("xzCoords", String, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    self.r = rospy.Rate(30)


  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      imageprocessor2 = Image_processes()
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    # xzCentres = imageprocessor2.Contours(xzcontours)

    # image_copy = self.cv_image2.copy()
    # for c in xzCentres:
    #     cv2.circle(image_copy, (c[0], c[1]), 2, (255, 255, 255), -1)
    #     cv2.putText(image_copy, "centroid", (c[0] - 25, c[1] - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    # cv2.imshow("Centriods",image_copy)
    # cv2.waitKey(1)

    # Publish the results
    try: 
      xzCentres = imageprocessor2.imProcess(self.cv_image2)
      self.imxz.publish(json.dumps(xzCentres))
      self.r.sleep()
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

