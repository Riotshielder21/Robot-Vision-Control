import roslib
import sys
import rospy
import cv2
import numpy as np

class Image_processes:

    def _init_(self):
        return

    def Contours(self, contours):
        DicCentres = {}
        # calculate moments for each contour
        if (contours['Green']!=[]):
                M = cv2.moments(contours['Green'][0])
                # calculate y,z coordinate of center
                if M["m00"] != 0:
                      cX = int(M["m10"] / M["m00"])
                      cY = int(M["m01"] / M["m00"])
                else:
                      cX, cY = 0, 0
                DicCentres['Green']=[cX,cY]

        if (contours['Yellow']!=[]):
                M = cv2.moments(contours['Yellow'][0])
                # calculate y,z coordinate of center
                if M["m00"] != 0:
                      cX = int(M["m10"] / M["m00"])
                      cY = int(M["m01"] / M["m00"])
                else:
                      cX, cY = 0, 0
                DicCentres['Yellow']=[cX,cY]

        if (contours['Blue']!=[]):
                M = cv2.moments(contours['Blue'][0])
                # calculate y,z coordinate of center
                if M["m00"] != 0:
                      cX = int(M["m10"] / M["m00"])
                      cY = int(M["m01"] / M["m00"])
                else:
                      cX, cY = 0, 0
                DicCentres['Blue']=[cX,cY]

        if (contours['Red']!=[]):
                M = cv2.moments(contours['Red'][0])
                # calculate y,z coordinate of center
                if M["m00"] != 0:
                      cX = int(M["m10"] / M["m00"])
                      cY = int(M["m01"] / M["m00"])
                else:
                      cX, cY = 0, 0
                DicCentres['Red']=[cX,cY]

        return DicCentres
        
#----------------------------------------------------------------------------------------------------------        
 
    def anglesVis1(self, centres):
        
        # needs to be adjusted to calculate angles for the 3 dimensions
        #   key      x    y    z
        #{'Green': {347, 350, 536}, 'Yellow': [347, 350, 431], 'Blue': [347, 350, 348], 'Red': [347, 350, 275]}
        if centres['Blue'] == {}:
                joint2 = -1.57
        else:
                if ('z' in centres ['Yellow'] and 'z' in centres['Blue']):
                        if (centres['Yellow']['x']-centres['Blue']['x']) >= 1  or (centres['Yellow']['x']-centres['Blue']['x']) <= -1:
                                joint2 = np.arctan2(round((centres['Yellow']['z']-centres['Blue']['z'])/10)*10, round((centres['Yellow']['x']-centres['Blue']['x'])/10)*10)                     
                                if joint2 < 0:
                                        joint2 += 2.35   
                                else:
                                        joint2 -= 2.35   
                                #print(joint2)
                        else:
                                joint2 = 0
                                #print(joint2)

        #link 2 angle, yellow to blue
        if centres['Blue'] == {}:
                joint3 = 0
        else:
                if ('z' in centres ['Yellow'] and 'z' in centres['Blue']):   
                        if (centres['Yellow']['y']-centres['Blue']['y'])  >= 2 or (centres['Yellow']['y']-centres['Blue']['y']) <= -2:
                                joint3 = np.arctan2((centres['Yellow']['z']-centres['Blue']['z']),(centres['Yellow']['y']-centres['Blue']['y'])) - joint2
                                #print(joint3)
                        else:
                                joint3 = 0
                                #print(joint3)

        #link 3 angle, blue to red 
        if centres['Blue'] == {}:
                joint4 = 0
        else:
                if ('z' in centres ['Blue'] and 'z' in centres['Red']):    
                        if (centres['Blue']['x']-centres['Red']['x'])  >= 2 or (centres['Blue']['y']-centres['Red']['y']) <= -2:
                                joint4 = np.arctan2((centres['Blue']['z']-centres['Red']['z']),(centres['Blue']['x']-centres['Red']['x'])) - joint3 - joint2
                                #print(joint4)
                        else:
                                joint4 = 0
                                #print(joint4)

        return np.array([0, joint2, joint3, joint4])

    def anglesVis2(self, centres):
            return 0

#----------------------------------------------------------------------------------------------------------
#----------------------------------------------------------------------------------------------------------       

        # Perform image processing, green base joint not required
    def imProcess(self, image):
    
        green_u = (20,256,20)
        green_l = (0,50,0)
        blue_u = (256,20,20)
        blue_l = (50,0,0)
        red_u = (20,20,256)
        red_l = (0,0,50)
        yellow_u = (20,256,256)
        yellow_l = (0,50,50)
        
        maskG = cv2.inRange(image, green_l, green_u)
        maskY = cv2.inRange(image, yellow_l, yellow_u)
        maskB = cv2.inRange(image, blue_l, blue_u)
        maskR = cv2.inRange(image, red_l, red_u)

        
        maskJ1 = cv2.cvtColor(maskG,cv2.COLOR_BGR2RGB)
        maskJ2 = cv2.cvtColor(maskY,cv2.COLOR_BGR2RGB)
        maskJ3 = cv2.cvtColor(maskB,cv2.COLOR_BGR2RGB)
        maskJ4 = cv2.cvtColor(maskR,cv2.COLOR_BGR2RGB)
        
        Green_frame = image & maskJ1
        Yellow_frame = image & maskJ2 
        Blue_frame = image & maskJ3 
        Red_frame = image & maskJ4
      
        Gframe_gray = cv2.cvtColor(Green_frame, cv2.COLOR_RGB2GRAY)
        Yframe_gray = cv2.cvtColor(Yellow_frame, cv2.COLOR_RGB2GRAY)
        Bframe_gray = cv2.cvtColor(Blue_frame, cv2.COLOR_RGB2GRAY)
        Rframe_gray = cv2.cvtColor(Red_frame, cv2.COLOR_RGB2GRAY)
        
        Gret, Gjoint_thresh = cv2.threshold(Gframe_gray, 1, 255, cv2.THRESH_BINARY)
        Yret, Yjoint_thresh = cv2.threshold(Yframe_gray, 1, 255, cv2.THRESH_BINARY)
        Bret, Bjoint_thresh = cv2.threshold(Bframe_gray, 1, 255, cv2.THRESH_BINARY)
        Rret, Rjoint_thresh = cv2.threshold(Rframe_gray, 1, 255, cv2.THRESH_BINARY)
        
        Gcontours, hierarchy = cv2.findContours(Gjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        Ycontours, hierarchy = cv2.findContours(Yjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        Rcontours, hierarchy = cv2.findContours(Bjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        Bcontours, hierarchy = cv2.findContours(Rjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        contourDic = {"Green": Gcontours,"Yellow": Ycontours,"Blue": Rcontours,"Red": Bcontours}

        #im_copy = image.copy()
        #cv2.drawContours(im_copy, Gcontours, -1, (255, 255, 255), 2, cv2.LINE_AA)
        #cv2.imshow('Contoured', im_copy)
        #cv2.waitKey(10000)

        return contourDic

    
