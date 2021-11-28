#!/usr/bin/env python3

import json
from typing import Dict

from numpy.lib.function_base import angle
import roslib
import sys
import rospy
import cv2
import numpy as np
import Process
from Process import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):

        self.CentresDict = {}

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        self.yzcoord = rospy.Subscriber("/yzCoords", String, self.callbackyz)
        self.xzcoord = rospy.Subscriber("/xzCoords", String, self.callbackxz)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher(
            "joint_angles", Float64MultiArray, queue_size=1)
        rate = rospy.Rate(5)  # 5hz
        # record the beginning time
        self.time_trajectory = rospy.get_time()
        while not rospy.is_shutdown():
            #print("Sending")
            self.anglesPublish(self.CentresDict)
            rate.sleep()

# --------------------------------------------------------------------------------------------------------------
    def callbackyz(self, data):

        try:
            self.CentresDict['yz'] = json.loads(data.data)

        except CvBridgeError as e:
            print(e)

    def callbackxz(self, data):

        # change te value of self.joint.data to your estimated value from the images
        # Publish the results
        try:
            self.CentresDict['xz'] = json.loads(data.data)

        except CvBridgeError as e:
            print(e)

    def anglesPublish(self, coords):
        if 'xz' in self.CentresDict and 'yz' in self.CentresDict:
            #print("Both Joints found")
            im = Image_processes()
            matched = im.matchCoords(coords)
            #print(matched)
            Angles = Float64MultiArray()
            Angles.data = im.anglesVis1(matched)
            print(Angles.data)
            self.joints_pub.publish(Angles)
        
    def calculate(self):
        ic = image_converter()
        if (('xz' in self.CentresDict) and ('yz' in self.CentresDict)):
            ic.anglesPublish(self.CentresDict)
        

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
