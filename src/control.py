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

class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
                # initialize the bridge between openCV and ROS
        rospy.init_node('my_broadcaster')
    
        self.b = TransformBroadcaster()
    
        translation = (0.0, 0.0, 0.0)
        rotation = (0.0, 0.0, 0.0, 1.0)
        rate = rospy.Rate(5)  # 5hz
        
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
        
    def callback(self, data):
        print("callback " + str(data.name))
        print(data.position)
        a1 = data.position[0]
        a2 = data.position[2]
        a3 = data.position[3]
        print([a1,a2,a3])
        frame0 = np.array(
        [[math.cos(a1),-math.sin(a1),0,0],
        [math.sin(a1),math.cos(a1),0,0],
        [0,0,1,0],
        [0,0,0,1]])
        #Rot x
        frame1 = np.array(
        [[1,0,0,0],
        [0,math.cos(a2),-math.sin(a2),0],
        [0,math.sin(a2),math.cos(a2),4],
        [0,0,0,1]])
        # #Rot y
        frame2 = np.array(
        [[math.cos(a3),0,math.sin(a3),0],
        [0,1,0,0],
        [-math.sin(a3),0,math.cos(a3),3.2],
        [0,0,0,1]])
        frame3= np.array(
        [[1,0,0,0],
        [0,1,0,0],
        [0,0,1,2.8],
        [0,0,0,1]])
        #Rot Z
        # frame1 = np.array(
        # [[math.cos(a1),-math.sin(a1),0,0],
        # [math.sin(a1),math.cos(a1),0,0],
        # [0,0,1,4],
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
        end = frame3@frame2@frame1@frame0
        translation = (end[0][3], end[1][3], end[2][3])
        print(translation)
        rotation = (0.0, 0.0, 0.0, 1.0)   
        self.b.sendTransform(translation, rotation, Time.now(), 'memes', 'base')
        # print(pos)
        # print(frame3* frame2* frame1)
        # let frame1 = np.array[]
        # Receive the image
    # # Define a circular trajectory
    # def trajectory(self):
    #     # get current time
    #     cur_time = np.array([rospy.get_time() - self.time_trajectory])
    #     x_d = float(6 * np.cos(cur_time * np.pi / 100))
    #     y_d = float(6 + np.absolute(1.5 * np.sin(cur_time * np.pi / 100)))
    #     return np.array([x_d, y_d])

    # # Receive data, process it, and publish
    # def callback(self, data):
    #     # Receive the image
    #     try:
    #         yz_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #         xz_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    #     except CvBridgeError as e:
    #         print(e)

       

    #     # Perform image processing task (your code goes here)
    #     # The image is loaded as cv_imag

        
    #     blue_u = (256,20,20)
    #     blue_l = (50,0,0)
    #     green_u = (20,256,20)
    #     green_l = (0,50,0)
    #     red_u = (20,20,256)
    #     red_l = (0,0,50)
    #     yellow_u = (20,256,256)
    #     yellow_l = (0,50,50)
        
    #     yzmaskY = cv2.inRange(yz_image1, yellow_l, yellow_u)
    #     yzmaskB = cv2.inRange(yz_image1, blue_l, blue_u)
    #     yzmaskG = cv2.inRange(yz_image1, green_l, green_u)
    #     yzmaskR = cv2.inRange(yz_image1, red_l, red_u)
    #     yzmaskJ1 = cv2.cvtColor(yzmaskY,cv2.COLOR_BGR2RGB)
    #     yzmaskJ2 = cv2.cvtColor(yzmaskB,cv2.COLOR_BGR2RGB)
    #     yzmaskJ3 = cv2.cvtColor(yzmaskG,cv2.COLOR_BGR2RGB)
    #     yzmaskJ4 = cv2.cvtColor(yzmaskR,cv2.COLOR_BGR2RGB)
        
    #     full_frame = yz_image1 & ( yzmaskJ1 | yzmaskJ2 | yzmaskJ3 | yzmaskJ4)
        
    #     frame_gray = cv2.cvtColor(full_frame, cv2.COLOR_RGB2GRAY)
    #     ret, joint_thresh = cv2.threshold(frame_gray, 1, 255, cv2.THRESH_BINARY)

    #     contours, hierarchy = cv2.findContours(joint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
    #     #image_copy = cv_image.copy()
      
    #     #cv2.drawContours(image_copy, contours, -1, (255, 255, 255), 2, cv2.LINE_AA)
    #     #cv2.imshow('Contoured', cv_image)
    #     #cv2.waitKey(10000)
        
    #     jointCentres = []
        
    #     for c in contours:
    #           # calculate moments for each contour
    #           M = cv2.moments(c)
    #           # calculate x,y coordinate of center
    #           if M["m00"] != 0:
    #                 cX = int(M["m10"] / M["m00"])
    #                 cY = int(M["m01"] / M["m00"])
    #           else:
    #                 cX, cY = 0, 0
              
    #           #3m = 93 pixels | 1m = 31 pixels
    #           jointCentres.append([cX/31,cY/31])
    #           #cv2.circle(image_copy, (cX, cY), 2, (255, 255, 255), -1)
    #           #cv2.putText(image_copy, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
              
        
    #     if (jointCentres[0][0]-jointCentres[1][0]) != 0:
    #           link1 = np.arctan2((jointCentres[0][1]-jointCentres[0][1])/(jointCentres[0][0]-jointCentres[1][0]))
    #     else:
    #           link1 = 0
    #     if (jointCentres[1][0]-jointCentres[2][0]) != 0:
    #           link2 = np.arctan2((jointCentres[1][1]-jointCentres[2][1])/(jointCentres[1][0]-jointCentres[2][0]))-link1
    #     else:
    #           link2 = 0
    #     if (jointCentres[2][0]-jointCentres[3][0]) != 0:
    #           link3 = np.arctan2((jointCentres[2][1]-jointCentres[3][1])/(jointCentres[2][0]-jointCentres[3][0])) -link1 - link2
    #     else:
    #           link3 = 0
        
    #     #cv2.imshow('Centroids', image_copy)
    #     #cv2.waitKey(100)

    #     #cv2.destroyAllWindows()
        
    #     4###########################################################################

    #     # change te value of self.joint.data to your estimated value from the images
    #     self.joints = Float64MultiArray()
    #     self.joints.data = np.array([link1,link2, link3])
    #     #  self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
    #     #  self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
    #     #  self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))
        
    #     # publish the estimated position of robot end-effector
    #     # x_e_image = np.array([0, 0, 0])
    #     # self.end_effector=Float64MultiArray()
    #     # self.end_effector.data= x_e_image

    #     # send control commands to joints
    #     # self.joint1=Float64()
    #     # self.joint1.data= q_d[0]
    #     # self.joint2=Float64()
    #     # self.joint2.data= q_d[1]
    #     # self.joint3=Float64()
    #     # self.joint3.data= q_d[2]

    #     # Publishing the desired trajectory on a topic named trajectory
    #     # x_d = self.trajectory()    # getting the desired trajectory
    #     # self.trajectory_desired= Float64MultiArray()
    #     # self.trajectory_desired.data=x_d

    #     # Publish the results - the images are published under a topic named "image_topic" and calculated joints angles are published under a topic named "joints_pos"
    #     try:
    #         self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #         self.joints_pub.publish(self.joints)
           
    #         # self.trajectory_pub.publish(self.trajectory_desired)
    #         # self.end_effector_pub.publish(self.end_effector)
    #         # self.robot_joint1_pub.publish(self.joint1)
    #         # self.robot_joint3_pub.publish(self.joint3)
    #         # self.robot_joint3_pub.publish(self.joint4)
    #     except CvBridgeError as e:
    #         print(e)


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
