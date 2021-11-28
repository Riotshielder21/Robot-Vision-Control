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
    self.xzCentres = None
    self.cv_image2 = None
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send xz coordinates
    self.imxz = rospy.Publisher("xzCoords", String, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()
    rate = rospy.Rate(5)  # 5hz
    # record the beginning time
    while not rospy.is_shutdown():
        #print("Sending")
        self.showimg()
        rate.sleep()

  def showimg(self):
          # yzCentres = imageprocessor1.Contours(yzcontours)
    if self.xzCentres is None or self.cv_image2 is None:
      return
  
    cv2.imshow("Image 2",self.cv_image2)
    cv2.waitKey(1)
    
  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      imageprocessor2 = Image_processes()
      cv_image2_ps = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    try: 
      xzCentres,self.cv_image2 = imageprocessor2.imProcess(cv_image2_ps)
      self.xzCentres = xzCentres
      self.imxz.publish(json.dumps(xzCentres))
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


