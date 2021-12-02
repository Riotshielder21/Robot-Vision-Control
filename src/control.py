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
from rospy import Time 
import math
from tf.transformations import quaternion_matrix
from tf.transformations import quaternion_from_matrix


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
                # initialize the bridge between openCV and ROS
        rospy.init_node('my_broadcaster')
    
        self.b = TransformBroadcaster()
    
        translation = (0.0, 0.0, 0.0)
        rotation = (0.0, 0.0, 0.0, 1.0)
        rate = rospy.Rate(50)  # 5hz
        
        x, y = 0.0, 0.0
        rospy.Subscriber("/robot/joint_states", JointState, self.callback) 
        while not rospy.is_shutdown():
            print("Sending")
            if x >= 2:
                x, y = 0.0, 0.0 
            
            x += 0.1
            y += 0.1
            
            translation = (x, y, 0.0)
            
            
            self.b.sendTransform(translation, rotation, Time.now(), 'memes', 'base')
            rate.sleep()
    def frame_to_pos(self, frame):
        return (frame[0][3], frame[1][3], frame[2][3])
    def frame_to_rot(self, frame):
        M1 = frame
        # q = [x1, y1, z1, 1.0]   
        r = np.math.sqrt(float(1)+M1[0,0]+M1[1,1]+M1[2,2])*0.5
        i = (M1[2,1]-M1[1,2])/(4*r)
        j = (M1[0,2]-M1[2,0])/(4*r)
        k = (M1[1,0]-M1[0,1])/(4*r)
        rotation = (i,j,k,r)
        return rotation
    def callback(self, data):
        print("callback " + str(data.name))
        print(data.position)
        a1 = data.position[0]
        a2 = data.position[2]
        a3 = data.position[3]


        # #Rot Z
        # frame1 = np.array(
        # [[math.cos(a1),-math.sin(a1),0,0],
        # [math.sin(a1),math.cos(a1),0,0],
        # [0,0,1,4.5],
        # [0,0,0,1]])
        # #Rot x
        # frame2 = np.array(
        # [[1,0,0,0],
        # [0,math.cos(a2),-math.sin(a2),0],
        # [0,math.sin(a2),math.cos(a2),3.2],
        # [0,0,0,1]])
        # #Rot y
        # frame3 = np.array(
        # [[math.cos(a3),0,math.sin(a3),0],
        # [0,1,0,0],
        # [-math.sin(a3),0,math.cos(a3),2.8],
        # [0,0,0,1]])
        # end = frame3@frame2@frame1
        # translation = self.frame_to_pos(end)
        # rotation = self.frame_to_rot(end)
        # print(translation)
        arm2_len = 2.8
        arm1_len = 3.2
        arm0_len = 4 + 0.5 #Offset for base of robot
        x1 = arm2_len*math.sin(a1)*math.cos(a3)*math.sin(a2) + arm2_len*math.cos(a1)*math.sin(a3) + arm1_len*math.sin(a1)*math.sin(a2)
        y1 = arm2_len*math.sin(a1)*math.sin(a3)-math.cos(a1)*math.sin(a2)*(arm2_len*math.cos(a3)+arm1_len)
        z1 = (arm2_len*math.cos(a3)+arm1_len)*math.cos(a2)+arm0_len
        translation = [x1,y1,z1]
        rotation = [0,0,0,1]
        # self.b.sendTransform(self.frame_to_pos(frame1), self.frame_to_rot(frame1), Time.now(), 't1', 't0')
        # self.b.sendTransform(self.frame_to_pos(frame2), self.frame_to_rot(frame2), Time.now(), 't2', 't1')
        # self.b.sendTransform(self.frame_to_pos(frame3), self.frame_to_rot(frame3), Time.now(), 't3', 't2')
        self.b.sendTransform(translation, rotation, Time.now(), 'memes', 'base')



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
