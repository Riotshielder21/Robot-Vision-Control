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
                        joint2 = np.pi/2-np.arctan2((centres['Yellow']['z']-centres['Blue']['z']),(centres['Yellow']['x']-centres['Blue']['x']))                   
                        #print(joint2)
                else:
                        joint2 = 0
                        #print(joint2)

                #link 2 angle, yellow to blue
                #print("z diff: "+str(centres['Yellow']['z']-centres['Blue']['z']))
     
                if (centres['Yellow']['z']-centres['Blue']['z']) !=0:
                        joint3 = np.pi/2-np.arctan2((centres['Yellow']['z']-centres['Blue']['z']),(centres['Yellow']['y']-centres['Blue']['y']))
                        #print(joint3)
                else:
                        joint3 = 1.4
                        #print(joint3)

                #link 3 angle, blue to red 
                #print("z diff: "+str(centres['Blue']['z']-centres['Red']['z'])) 
                if (centres['Blue']['z']-centres['Red']['z']) !=0:
                        joint4 = np.pi/2-np.arctan2((centres['Blue']['z']-centres['Red']['z']),(centres['Blue']['x']-centres['Red']['x'])) -  joint2
                        #print(joint4)
                else:
                        joint4 = 0
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
                climits = [[green_l,green_u],[yellow_l,yellow_u],[blue_l,blue_u],[red_l,red_u]]
                
                masks = [ cv2.inRange(image, climit[0], climit[1]) for climit in climits]
              
                maskJs = [cv2.cvtColor(mask,cv2.COLOR_BGR2RGB) for mask in masks]
              
                frames = [(image&maskJ) for maskJ in maskJs]
                
                gray_frames = [cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY) for frame in frames]
                
                jThreshes = [cv2.threshold(gray_frame, 1, 255, cv2.THRESH_BINARY) for gray_frame in gray_frames]
                
                jcontours = [cv2.findContours(jthresh[1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) for jthresh in jThreshes]
               
                cords = []
                radiuslist = []
                for jcontour in jcontours:
                        # print(jcontour)
                        try:    
                                Gradius = 0
                                (Gx,Gy),Gradius = cv2.minEnclosingCircle(self.mergeContors(jcontour[0]))
                                radiuslist.append(Gradius)
                                # print(Gradius)
                                if Gradius < 2: #Filter out single pixel showing
                                        cords.append([-1,-1])
                                else:
                                        cords.append([Gx,Gy])
                                        
                        except:
                                cords.append([-1,-1])
                                radiuslist.append(0)

                contourDic = {"Green": {'x':cords[0][0],'y':cords[0][1]},"Yellow": {'x':cords[1][0],'y':cords[1][1]},"Blue": {'x':cords[2][0],'y':cords[2][1]},"Red": {'x':cords[3][0],'y':cords[3][1]}}
                
                im_copy = image.copy()
                

                for i in range(len(cords)):
                        cv2.circle(im_copy, (int(cords[i][0]), int(cords[i][1])), 2, (255, 255, 255), -1)
                        cv2.putText(im_copy, list(contourDic.keys())[i], (int(cords[i][0]) - 50, int(cords[i][1]) - 25),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.circle(im_copy,(int(cords[i][0]), int(cords[i][1])),int(radiuslist[i]),(0,255,0),1)


                return contourDic, im_copy
        def mergeContors(self, ctrs):
                list_of_pts = []
                for c in ctrs:
                        for e in c:
                                list_of_pts.append(e)
                ctr = np.array(list_of_pts).reshape((-1,1,2)).astype(np.int32)
                ctr = cv2.convexHull(ctr)
                return ctr

        def matchCoords(self, centres):

                # the yz coordinates are still labeled as x and y due to function reusability but x = y and y = z in the yz case

                matchCoords = {}

                yzcenter = {}
                xzcenter = {}
                for c in centres['yz']:
                        yzcenter[c] = {}
                        yzcenter[c]['x'] = -1
                        yzcenter[c]['y'] = centres['yz'][c]['x']
                        yzcenter[c]['z'] = centres['yz'][c]['y']
                for c in centres['xz']:
                        xzcenter[c] = {}
                        xzcenter[c]['x'] = centres['xz'][c]['x']
                        xzcenter[c]['y'] = -1
                        xzcenter[c]['z'] = centres['xz'][c]['y']
                for colour in yzcenter:
                        #print(colour)
                        matchCoords[colour] = {}
                        for axis in yzcenter[colour]:
                                #print(axis)
                                out = -1
                                cord1 = yzcenter[colour][axis]
                                cord2 = xzcenter[colour][axis]
                                if cord1 >= 0 and cord2 >= 0:
                                        out = np.mean([cord1,cord2])
                                else:
                                        out = np.max([cord1, cord2])
                                if out > 0:
                                        matchCoords[colour][axis] = round(out/10)*10
                                else:
                                        print("Warning missing data")

                if centres['xz']['Yellow']['y'] == -1:
                        matchCoords["Yellow"]['z'] = centres['yz']['Yellow']['y']
                else:
                        matchCoords["Yellow"]['z'] = centres['xz']['Yellow']['y']
                   
                if centres['xz']['Blue']['y'] == -1: 
                        matchCoords["Blue"]['z'] = centres['yz']['Blue']['y']
                else:
                        matchCoords["Blue"]['z'] = centres['xz']['Blue']['y']
                if  matchCoords["Blue"]['y'] == -1: 
                        matchCoords["Blue"]['y'] =  matchCoords["Yellow"]['y']
                if  matchCoords["Blue"]['x'] == -1: 
                        matchCoords["Blue"]['x'] =  matchCoords["Yellow"]['x']
                 
                if centres['xz']['Red']['y'] == -1:
                        matchCoords["Red"]['z'] = centres['yz']['Red']['y']
                else:
                        matchCoords["Red"]['z'] = centres['xz']['Red']['y']

                if matchCoords["Red"]['x'] == -1:
                        matchCoords["Red"]['x'] = matchCoords["Blue"]['x']
                if matchCoords["Red"]['y'] == -1:
                        matchCoords["Red"]['y'] = matchCoords["Blue"]['y']
                #print(matchCoords)
                return matchCoords
