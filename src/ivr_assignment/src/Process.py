import roslib
import sys
import rospy
import cv2
import numpy as np

class Image_processes:

    def _init_(self):
        return
        
    def Contours(self, contours):
        Centres = []	
        for c in contours:
            # calculate moments for each contour
            M = cv2.moments(c)
            # calculate y,z coordinate of center
            if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
            else:
                    cX, cY = 0, 0

            #3m = 93 pixels | 1m = 31 pixels
            Centres.append([cX,cY])
        return Centres
        
#----------------------------------------------------------------------------------------------------------     

    def XYContours(self, xzcontours, yzcontours):
        xCentres = []
        yCentres = []
        for cx in xzcontours:
        
          # calculate moments for each xz contour
          M = cv2.moments(cx)
          # calculate y,z coordinate of center
          if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cZ = int(M["m01"] / M["m00"])
          else:
                cX, cY = 0, 0
          xCentres.append([cX])
        for cy in yzcontours:
            
          # calculate moments for each yz contour
          M = cv2.moments(cy)
          # calculate y,z coordinate of center
          if M["m00"] != 0:
                cY = int(M["m10"] / M["m00"])
                cZ = int(M["m01"] / M["m00"])
          else:
                cY, cZ = 0, 0
          #3m = 93 pixels | 1m = 31 pixels
          yCentres.append([cY])
        Centres = []
        return Centres
   	

#----------------------------------------------------------------------------------------------------------        
 
#link 1 angle, green to yellow
    def angles(self, centres):
        
        if (centres[0][0]-centres[1][0]) != 0:
                joint2 = np.arctan2((centres[0][1]-centres[0][1])/(centres[0][0]-centres[1][0]))
                print(joint2)
        else:
                joint2 = 0

        #link 2 angle, yellow to blue     
        if (centres[1][0]-centres[2][0]) != 0:
                joint3 = np.arctan2((centres[1][1]-centres[2][1])/(centres[1][0]-centres[2][0]))-joint2
                print(joint3)
        else:
                joint3 = 0

        #link 3 angle, blue to red      
        if (centres[2][0]-centres[3][0]) != 0:
                joint4 = np.arctan2((centres[2][1]-centres[3][1])/(centres[2][0]-centres[3][0])) -joint2 - joint3
                print(joint4)
        else:
                joint4 = 0

        return np.array[joint2, joint3, joint4]

#----------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------       

        # Perform image processing, green base joint not required
    def imProcess(self, image):
    
        #green_u = (20,256,20)
        #green_l = (0,50,0)
        blue_u = (256,20,20)
        blue_l = (50,0,0)
        red_u = (20,20,256)
        red_l = (0,0,50)
        yellow_u = (20,256,256)
        yellow_l = (0,50,50)
        
        #maskG = cv2.inRange(image, green_l, green_u)
        maskY = cv2.inRange(image, yellow_l, yellow_u)
        maskB = cv2.inRange(image, blue_l, blue_u)
        maskR = cv2.inRange(image, red_l, red_u)

        
        #maskJ1 = cv2.cvtColor(maskG,cv2.COLOR_BGR2RGB)
        maskJ2 = cv2.cvtColor(maskY,cv2.COLOR_BGR2RGB)
        maskJ3 = cv2.cvtColor(maskB,cv2.COLOR_BGR2RGB)
        maskJ4 = cv2.cvtColor(maskR,cv2.COLOR_BGR2RGB)
        
        full_frame = image & (maskJ2 | maskJ3 | maskJ4)
      
        frame_gray = cv2.cvtColor(full_frame, cv2.COLOR_RGB2GRAY)
        
        ret, joint_thresh = cv2.threshold(frame_gray, 1, 255, cv2.THRESH_BINARY)
        
        contours, hierarchy = cv2.findContours(joint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        image_copy = image.copy()
        cv2.drawContours(image_copy, contours, -1, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imshow('Contoured', image)
        cv2.waitKey(10000)
        
        Centres = self.Contours(contours)