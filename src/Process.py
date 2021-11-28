import roslib
import sys
import rospy
import cv2
import numpy as np

class Image_processes:

        def _init_(self):
                return

        #def Contours(self, contours):
        #         DicCentres = {}
        #         # calculate moments for each contour
        #         if (contours['Green']!=[]):
        #                 M = cv2.moments(contours['Green'][0])
        #                 # calculate y,z coordinate of center
        #                 if M["m00"] != 0:
        #                       cX = int(M["m10"] / M["m00"])
        #                       cY = int(M["m01"] / M["m00"])
        #                 else:
        #                       cX, cY = 0, 0
        #                 DicCentres['Green']=[cX,cY]

        #         if (contours['Yellow']!=[]):
        #                 M = cv2.moments(contours['Yellow'][0])
        #                 # calculate y,z coordinate of center
        #                 if M["m00"] != 0:
        #                       cX = int(M["m10"] / M["m00"])
        #                       cY = int(M["m01"] / M["m00"])
        #                 else:
        #                       cX, cY = 0, 0
        #                 DicCentres['Yellow']=[cX,cY]

        #         if (contours['Blue']!=[]):
        #                 M = cv2.moments(contours['Blue'][0])
        #                 # calculate y,z coordinate of center
        #                 if M["m00"] != 0:
        #                       cX = int(M["m10"] / M["m00"])
        #                       cY = int(M["m01"] / M["m00"])
        #                 else:
        #                       cX, cY = 0, 0
        #                 DicCentres['Blue']=[cX,cY]

        #         if (contours['Red']!=[]):
        #                 M = cv2.moments(contours['Red'][0])
        #                 # calculate y,z coordinate of center
        #                 if M["m00"] != 0:
        #                       cX = int(M["m10"] / M["m00"])
        #                       cY = int(M["m01"] / M["m00"])
        #                 else:
        #                       cX, cY = 0, 0
        #                 DicCentres['Red']=[cX,cY]

        #         return DicCentres
                
#----------------------------------------------------------------------------------------------------------        
 
        def anglesVis1(self, centres):
                
                # needs to be adjusted to calculate angles for the 3 dimensions
                #   key      x    y    z
                #{'Green': {347, 350, 536}, 'Yellow': [347, 350, 431], 'Blue': [347, 350, 348], 'Red': [347, 350, 275]}
                #print("x diff: "+str(centres['Yellow']['x']-centres['Blue']['x']))
                if (centres['Yellow']['z']-centres['Blue']['z']) != 0:
                        joint2 = np.arctan2((centres['Yellow']['z']-centres['Blue']['z']),(centres['Yellow']['x']-centres['Blue']['x']))                   
                        #print(joint2)
                else:
                        joint2 = 1.57
                        #print(joint2)

                #link 2 angle, yellow to blue
                #print("z diff: "+str(centres['Yellow']['z']-centres['Blue']['z']))
     
                if (centres['Yellow']['z']-centres['Blue']['z']) !=0:
                        joint3 = np.arctan2((centres['Yellow']['z']-centres['Blue']['z']),(centres['Yellow']['y']-centres['Blue']['y']))
                        #print(joint3)
                else:
                        joint3 = 1.57
                        #print(joint3)

                #link 3 angle, blue to red 
                #print("z diff: "+str(centres['Blue']['z']-centres['Red']['z'])) 
                if (centres['Blue']['z']-centres['Red']['z']) !=0:
                        joint4 = np.arctan2((centres['Blue']['z']-centres['Red']['z']),(centres['Blue']['x']-centres['Red']['x'])) -  joint2
                        #print(joint4)
                else:
                        joint4 = 1.57
                        #print(joint4)
                        
                print(centres)
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
                
                Gcontour, hierarchy = cv2.findContours(Gjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                Ycontour, hierarchy = cv2.findContours(Yjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                Bcontour, hierarchy = cv2.findContours(Bjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                Rcontour, hierarchy = cv2.findContours(Rjoint_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                
                try:
                        (Gx,Gy),Gradius = cv2.minEnclosingCircle(Gcontour[0])
                except:
                        Gx = -1
                        Gy = -1
                try:
                
                        (Yx,Yy),Yradius = cv2.minEnclosingCircle(Ycontour[0])
                except:
                        Yx = -1
                        Yy = -1
                try:

                        (Rx,Ry),Rradius = cv2.minEnclosingCircle(Rcontour[0])
                except:
                        Rx = -1
                        Ry = -1
                try:
                        (Bx,By),Bradius = cv2.minEnclosingCircle(Bcontour[0])
                except:
                        Bx = -1
                        By = -1

                contourDic = {"Green": {'x':Gx,'y':Gy},"Yellow": {'x':Yx,'y':Yy},"Blue": {'x':Bx,'y':By},"Red": {'x':Rx,'y':Ry}}


                #im_copy = image.copy()
                #cv2.circle(im_copy,(Gx,Gy),Gradius,(0,255,0),2)
                #cv2.drawContours(im_copy, Gcontour, -1, (255, 255, 255), 2, cv2.LINE_AA)
                #cv2.imshow('Contoured', im_copy)
                #cv2.waitKey(10000)

                return contourDic
        
        def matchCoords(self, centres):

                # the yz coordinates are still labeled as x and y due to function reusability but x = y and y = z in the yz case

                matchCoords = {}
                matchCoords['Green'] = {}
                matchCoords["Green"]['x'] = round(centres['xz']['Green']['x']/10)*10
                matchCoords["Green"]['y'] = round(centres['yz']['Green']['x']/10)*10
                matchCoords["Green"]['z'] = round(centres['yz']['Green']['y']/10)*10

                matchCoords['Yellow'] = {}
                if centres['xz']['Yellow']['y'] == -1:                 
                        matchCoords["Yellow"]['x'] = round(centres['xz']['Yellow']['x']/10)*10
                        matchCoords["Yellow"]['y'] = round(centres['yz']['Yellow']['x']/10)*10
                        matchCoords["Yellow"]['z'] = round(centres['yz']['Yellow']['y']/10)*10
                else:
                        matchCoords["Yellow"]['x'] = round(centres['xz']['Yellow']['x']/10)*10
                        matchCoords["Yellow"]['y'] = round(centres['yz']['Yellow']['x']/10)*10
                        matchCoords["Yellow"]['z'] = round(centres['xz']['Yellow']['y']/10)*10
                
                matchCoords['Blue'] = {}      
                if centres['xz']['Blue']['y'] == -1: 
                        matchCoords["Blue"]['x'] = -1
                        matchCoords["Blue"]['y'] = round(centres['yz']['Blue']['x']/10)*10
                        matchCoords["Blue"]['z'] = round(centres['yz']['Blue']['y']/10)*10
                else:
                        matchCoords["Blue"]['x'] = round(centres['xz']['Blue']['x']/10)*10
                        matchCoords["Blue"]['y'] = -1
                        matchCoords["Blue"]['z'] = round(centres['xz']['Blue']['y']/10)*10
                if  matchCoords["Blue"]['y'] == -1: 
                        matchCoords["Blue"]['y'] =  matchCoords["Yellow"]['y']
                if  matchCoords["Blue"]['x'] == -1: 
                        matchCoords["Blue"]['x'] =  matchCoords["Yellow"]['x']
        
                matchCoords['Red'] = {}            
                if centres['xz']['Red']['y'] == -1: 
                        matchCoords["Red"]['x'] = -1
                        matchCoords["Red"]['y'] = round(centres['yz']['Red']['x']/10)*10
                        matchCoords["Red"]['z'] = round(centres['yz']['Red']['y']/10)*10
                else:
                        matchCoords["Red"]['x'] = round(centres['xz']['Red']['x']/10)*10
                        matchCoords["Red"]['y'] = -1
                        matchCoords["Red"]['z'] = round(centres['xz']['Red']['y']/10)*10

                if matchCoords["Red"]['x'] == -1:
                        matchCoords["Red"]['x'] = matchCoords["Blue"]['x']
                if matchCoords["Red"]['y'] == -1:
                        matchCoords["Red"]['y'] = matchCoords["Blue"]['y']
                #print(matchCoords)
                return matchCoords
