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
            "joint_angles", Float64MultiArray, queue_size=10)

        # record the beginning time
        self.time_trajectory = rospy.get_time()

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

            if (('xz' in self.CentresDict) and ('yz' in self.CentresDict)):
                ic = image_converter()
                matched = ic.matchCoords(self.CentresDict)
                ic.anglesPublish(matched)

        except CvBridgeError as e:
            print(e)

    def anglesPublish(self, matched):
        im = Image_processes()
        Angles = Float64MultiArray()
        Angles.data = im.anglesVis1(matched)
        self.joints_pub.publish(Angles)

    def matchCoords(self, centres):
        matchCoords = {}
        if 'Green' in centres['xz']:
            if 'Green' in centres['yz']:
                if centres['yz']['Green'][1]:
                    matchCoords['Green'] = {}
                    matchCoords["Green"]['x'] = centres['xz']['Green'][0]
                    matchCoords["Green"]['y'] = centres['yz']['Green'][0]
                    matchCoords["Green"]['z'] = centres['yz']['Green'][1]
                else:
                    matchCoords['Green'] = {}
                    matchCoords["Green"]['x'] = centres['xz']['Green'][0]
                    matchCoords["Green"]['y'] = centres['yz']['Green'][0]
                    matchCoords["Green"]['z'] = centres['xz']['Green'][1]

        if 'Yellow' in centres['xz']:
            if 'Yellow' in centres['yz']:
                if centres['yz']['Yellow'][1]:
                    matchCoords['Yellow'] = {}
                    matchCoords["Yellow"]['x'] = centres['xz']['Yellow'][0]
                    matchCoords["Yellow"]['y'] = centres['yz']['Yellow'][0]
                    matchCoords["Yellow"]['z'] = centres['yz']['Yellow'][1]
                else:
                    matchCoords['Yellow'] = {}
                    matchCoords["Yellow"]['x'] = centres['xz']['Yellow'][0]
                    matchCoords["Yellow"]['y'] = centres['yz']['Yellow'][0]
                    matchCoords["Yellow"]['z'] = centres['xz']['Yellow'][1]
                
        if 'Blue' in centres['xz']:
            if 'Blue' in centres['yz']:
                if centres['yz']['Blue'][1]:
                    matchCoords['Blue'] = {}
                    matchCoords["Blue"]['x'] = centres['xz']['Blue'][0]
                    matchCoords["Blue"]['y'] = centres['yz']['Blue'][0]
                    matchCoords["Blue"]['z'] = centres['yz']['Blue'][1]
                else:
                    matchCoords['Blue'] = {}
                    matchCoords["Blue"]['x'] = centres['xz']['Blue'][0]
                    matchCoords["Blue"]['y'] = centres['yz']['Blue'][0]
                    matchCoords["Blue"]['z'] = centres['xz']['Blue'][1]

                    
        if 'Red' in centres['xz']:
            if 'Red' in centres['yz']:
                if centres['yz']['Red'][1]:
                    matchCoords['Red'] = {}
                    matchCoords["Red"]['x'] = centres['xz']['Red'][0]
                    matchCoords["Red"]['y'] = centres['yz']['Red'][0]
                    matchCoords["Red"]['z'] = centres['yz']['Red'][1]
                else:
                    matchCoords['Red'] = {}
                    matchCoords["Red"]['x'] = centres['xz']['Red'][0]
                    matchCoords["Red"]['y'] = centres['yz']['Red'][0]
                    matchCoords["Red"]['z'] = centres['xz']['Red'][1]
                                
        #print(matchCoords)
        return matchCoords
        

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
