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
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)

        self.yzcoord = rospy.Subscriber("/yzCoords", String, self.callbackyz)
        self.xzcoord = rospy.Subscriber("/xzCoords", String, self.callbackxz)

        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher(
            "joint_angles", Float64MultiArray, queue_size=10)

        self.CentresDict = {}
        self.matchedCoords = {}

        # record the beginning time
        self.time_trajectory = rospy.get_time()

# --------------------------------------------------------------------------------------------------------------
    def callbackyz(self, data):

        dictionaryDatayz = json.loads(data)

        try:

            self.CentresDict['yz'] = data['yz']

        except CvBridgeError as e:
            print(e)

    def callbackxz(self, data):

        dictionaryDataxz = json.loads(data)
        # change te value of self.joint.data to your estimated value from the images
        # Publish the results
        try:

            self.CentresDict['xz'] = data['xz']

        except CvBridgeError as e:
            print(e)

    def anglesPublish(self):
        im = Image_processes()
        Angles = im.anglesVis1(self.CentresDict)
        self.joints_pub.publish(Angles)

    def matchCoords(self, centresDict):
        if self.CentresDict['xz']['Green'] != []:
            if self.CentresDict['yz']['Green'] != []:
                self.matchCoords["Green"] = [self.CentresDict['xz']['Green'][0],
                                             self.CentresDict['yz']['Green'][0], 
                                             self.CentresDict['yz']['Green'][1]]
        if self.CentresDict['xz']['Yellow'] != []:
            if self.CentresDict['yz']['Yellow'] != []:
                self.matchCoords["Yellow"] = [self.CentresDict['xz']['Yellow'][0],
                                             self.CentresDict['yz']['Yellow'][0], 
                                             self.CentresDict['yz']['Yellow'][1]]
        if self.CentresDict['xz']['Blue'] != []:
            if self.CentresDict['yz']['Blue'] != []:
                self.matchCoords["Blue"] = [self.CentresDict['xz']['Blue'][0],
                                             self.CentresDict['yz']['Blue'][0], 
                                             self.CentresDict['yz']['Blue'][1]]

        if self.CentresDict['xz']['Red'] != []:
            if self.CentresDict['yz']['Red'] != []:
                self.matchCoords["Green"] = [self.CentresDict['xz']['Red'][0],
                                             self.CentresDict['yz']['Red'][0], 
                                             self.CentresDict['yz']['Red'][1]]
        print(self.matchCoords)
                # call the class


def main(args):
    ic = image_converter()

    try: 
        rospy.spin()
        ic.matchCoords()
        ic.anglesPublish()

    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
