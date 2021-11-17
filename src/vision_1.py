#!/usr/bin/env python3

import json
from typing import Dict
import roslib
import sys
import rospy
import cv2
import numpy as np
import Process
import image1
import image2
from Process import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
                # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        
        self.yzcoord = rospy.Subscriber("/YZCoords", json, self.callbackyz)
        self.xzcoord = rospy.Subscriber("/XZCoords", json, self.callbackxz)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joint_angles", Float64MultiArray, queue_size=10)
        
          # record the beginning time
        self.time_trajectory = rospy.get_time()

#--------------------------------------------------------------------------------------------------------------
     def callbackyz(self, data):

        dictionaryData = json.loads(data)

        #delay the callback or execution till two messages have passed to read

        # put here the matching of colours from XZ and YZ

        # in here put the function to call angles

         # Publish the results
        try:

            self.joints_pub.publish(self.joints)
           
        except CvBridgeError as e:
            print(e)
            
     def callbackxz(self, data):

          dictionaryData = json.loads(data)

          # change te value of self.joint.data to your estimated value from the images


        # Publish the results
        try:

            self.joints_pub.publish(self.joints)
           
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