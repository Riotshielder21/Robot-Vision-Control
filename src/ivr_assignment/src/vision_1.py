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


class image_converter:

    # Defines publisher and subscriber
    def __init__(self):
                # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize the node named image_processing
        rospy.init_node('image_processing', anonymous=True)
        # initialize a publisher to send messages to a topic named image_topic
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        # initialize a publisher to send joints' angular position to a topic called joints_pos
        self.joints_pub = rospy.Publisher("joints_pos", Float64MultiArray, queue_size=10)
        
        # initialize a subscriber to receive messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_subyz = rospy.Subscriber("/robot/camera1/image_raw", Image, self.callbackyz)
        self.image_subxz = rospy.Subscriber("/robot/camera2/image_raw", Image, self.callbackxz)

        # initialize a publisher to send robot end-effector position
        self.end_effector_pub = rospy.Publisher("end_effector_prediction", Float64MultiArray, queue_size=10)
        
        # initialize a publisher to send desired trajectory
        self.trajectory_pub = rospy.Publisher("trajectory", Float64MultiArray, queue_size=10)
        
        # initialize a publisher to send joints' angular position to the robot
        self.robot_joint1_pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.robot_joint2_pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.robot_joint3_pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.robot_joint4_pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        # record the beginning time
        self.time_trajectory = rospy.get_time()

#----------------------------------------------------------------------------------------------------------

    # Define a circular trajectory
    def trajectory(self):
        # get current time
        cur_time = np.array([rospy.get_time() - self.time_trajectory])
        x_d = float(6 * np.cos(cur_time * np.pi / 100))
        y_d = float(6 + np.absolute(1.5 * np.sin(cur_time * np.pi / 100)))
        return np.array([x_d, y_d])
        
#----------------------------------------------------------------------------------------------------------
    # Receive data, process it, and publish yz plane
    def callbackyz(self, data):
        # Receive the image
        try:
            yz_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
    # Receive data, process it, and publish xz plane
    def callbackxz(self, data):
	# Receive the image
	try:
	    xz_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
	    print(e)
#----------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------       

        # Perform image processing, green base joint not required
        # The image is loaded as cv_imag

        
        green_u = (20,256,20)
        green_l = (0,50,0)
        blue_u = (256,20,20)
        blue_l = (50,0,0)
        red_u = (20,20,256)
        red_l = (0,0,50)
        yellow_u = (20,256,256)
        yellow_l = (0,50,50)
        
        yzmaskG = cv2.inRange(yz_image1, green_l, green_u)
        yzmaskY = cv2.inRange(yz_image1, yellow_l, yellow_u)
        yzmaskB = cv2.inRange(yz_image1, blue_l, blue_u)
        yzmaskR = cv2.inRange(yz_image1, red_l, red_u)
        
        yzmaskJ1 = cv2.cvtColor(yzmaskG,cv2.COLOR_BGR2RGB)
        yzmaskJ2 = cv2.cvtColor(yzmaskY,cv2.COLOR_BGR2RGB)
        yzmaskJ3 = cv2.cvtColor(yzmaskB,cv2.COLOR_BGR2RGB)
        yzmaskJ4 = cv2.cvtColor(yzmaskR,cv2.COLOR_BGR2RGB)
        
        full_frameyz = yz_image & ( yzmaskJ1 | yzmaskJ2 | yzmaskJ3 | yzmaskJ4)
        
        frame_grayyz = cv2.cvtColor(full_frameyz, cv2.COLOR_RGB2GRAY)
        ret, joint_thresh = cv2.threshold(frame_grayyz, 1, 255, cv2.THRESH_BINARY)

        contours, hierarchy = cv2.findContours(joint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        #image_copy = cv_image.copy()
      
        #cv2.drawContours(image_copy, contours, -1, (255, 255, 255), 2, cv2.LINE_AA)
        #cv2.imshow('Contoured', cv_image)
        #cv2.waitKey(10000)
        
        jointCentres = []
        
 #----------------------------------------------------------------------------------------------------------
       
        for c in contours:
              # calculate moments for each contour
              M = cv2.moments(c)
              # calculate x,y coordinate of center
              if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
              else:
                    cX, cY = 0, 0
              
              #3m = 93 pixels | 1m = 31 pixels
              jointCentres.append([cX/31,cY/31])
              #cv2.circle(image_copy, (cX, cY), 2, (255, 255, 255), -1)
              #cv2.putText(image_copy, "centroid", (cX - 25, cY - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
   	
   	#link 1 = 4m
   	#joint 2 = link 2 = 0m
   	#link 2 = 3.2m
   	#link 3 = 2.8m
   	
#----------------------------------------------------------------------------------------------------------        
        
        #link 1 angle, green to yellow
        if (jointCentres[0][0]-jointCentres[1][0]) != 0:
              link1 = np.arctan2((jointCentres[0][1]-jointCentres[0][1])/(jointCentres[0][0]-jointCentres[1][0]))
              print(link1)
        else:
              link1 = 0
        
        #link 2 angle, yellow to blue     
        if (jointCentres[1][0]-jointCentres[2][0]) != 0:
              link2 = np.arctan2((jointCentres[1][1]-jointCentres[2][1])/(jointCentres[1][0]-jointCentres[2][0]))-link1
              print(link2)
        else:
              link2 = 0
        
        #link 3 angle, blue to red      
        if (jointCentres[2][0]-jointCentres[3][0]) != 0:
              link3 = np.arctan2((jointCentres[2][1]-jointCentres[3][1])/(jointCentres[2][0]-jointCentres[3][0])) -link1 - link2
              print(link3)
        else:
              link3 = 0
        
        #cv2.imshow('Centroids', image_copy)
        #cv2.waitKey(100)

        #cv2.destroyAllWindows()

#--------------------------------------------------------------------------------------------------------------
#--------------------------------------------------------------------------------------------------------------

        # change te value of self.joint.data to your estimated value from the images
        self.joints = Float64MultiArray()
        self.joints.data = np.array([link1,link2, link3])
        #  self.link1 = cv2.inRange(cv2.imread('link1.png', 1), (200, 200, 200), (255, 255, 255))
        #  self.link2 = cv2.inRange(cv2.imread('link2.png', 1), (200, 200, 200), (255, 255, 255))
        #  self.link3 = cv2.inRange(cv2.imread('link3.png', 1), (200, 200, 200), (255, 255, 255))
        
        # publish the estimated position of robot end-effector
        # x_e_image = np.array([0, 0, 0])
        # self.end_effector=Float64MultiArray()
        # self.end_effector.data= x_e_image

        # send control commands to joints
        # self.joint1=Float64()
        # self.joint1.data= q_d[0]
        # self.joint2=Float64()
        # self.joint2.data= q_d[1]
        # self.joint3=Float64()
        # self.joint3.data= q_d[2]

        # Publishing the desired trajectory on a topic named trajectory
        # x_d = self.trajectory()    # getting the desired trajectory
        # self.trajectory_desired= Float64MultiArray()
        # self.trajectory_desired.data=x_d

        # Publish the results - the images are published under a topic named "image_topic" and calculated joints angles are published under a topic named "joints_pos"
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.joints_pub.publish(self.joints)
           
            # self.trajectory_pub.publish(self.trajectory_desired)
            # self.end_effector_pub.publish(self.end_effector)
            # self.robot_joint1_pub.publish(self.joint1)
            # self.robot_joint3_pub.publish(self.joint3)
            # self.robot_joint3_pub.publish(self.joint4)
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