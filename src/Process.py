import roslib
import sys
import rospy
import cv2
import numpy as np
from tf import TransformBroadcaster
from sensor_msgs.msg import JointState
import rospy
from rospy import Time 
class Image_processes:

        def _init_(self):
                return
 
        def anglesVis1(self, centres):
                
                # joint3 first as it affects the joint2 position if set
                # link 3 angle, yellow yo blue 
                if (centres['Yellow']['z']-centres['Blue']['z'])>0:
                        joint3 = np.pi/2 + np.arctan2((centres['Blue']['z']-centres['Yellow']['z']),(centres['Yellow']['y']-centres['Blue']['y']))
                else:  
                        joint3 = 0
                
                if joint3 > 1.5707:
                        joint3 = joint3 - np.pi
                if joint3 < -1.5707:
                        joint3 = joint3 + np.pi

                #link2 angle, Yellow to blue
                if joint3 > 0.1:
                        if (centres['Yellow']['z']-centres['Red']['z']) != 0:
                                if round((centres['Yellow']['x']-centres['Red']['x']),2)>0:
                                        joint2 = -np.arctan2((centres['Red']['z']-centres['Yellow']['z']),(centres['Yellow']['x']-centres['Red']['x']))               
                                        print("joint3>0 and red to left of yellow xz")
                                elif round((centres['Yellow']['x']-centres['Red']['x']),2)<0:
                                        joint2 = np.arctan2((centres['Red']['z']-centres['Yellow']['z']),(centres['Red']['x']-centres['Yellow']['x']))               
                                        print("joint3>0 and red to right of yellow xz")
                                else: 
                                        joint2 = -1.57
                                        joint3 = -1.57
                        else:
                                joint2 = 0
                elif joint3 < -0.1:
                        if (centres['Yellow']['z']-centres['Red']['z']) != 0:
                                if round((centres['Yellow']['x']-centres['Red']['x']),2)>0:
                                        joint2 = np.arctan2((centres['Red']['z']-centres['Yellow']['z']),(centres['Yellow']['x']-centres['Red']['x']))                
                                        print("joint3<0 and red to right of yellow xz")
                                elif round((centres['Yellow']['x']-centres['Red']['x']),2)<0:
                                        joint2 = -np.arctan2((centres['Red']['z']-centres['Yellow']['z']),(centres['Yellow']['x']-centres['Red']['x']))               
                                        print("joint3<0 and red to right of yellow xz")
                                else:
                                        joint2 = 0
                        else: 
                                joint2 = 0

                else:
                        if (centres['Yellow']['z']-centres['Blue']['z']) !=0:
                                joint2 = np.pi/2 + np.arctan2((centres['Blue']['z']-centres['Yellow']['z']),(centres['Blue']['x']-centres['Yellow']['x']))                
                                print("joint 3 = 0")
                        else:
                                joint2 = 1.57
                                print("joint 3 = 0 and z diff = 0")
                if joint2 > 1.5707:
                        joint2 = joint2 - np.pi
                if joint2 < -1.5707:
                        joint2 = joint2 + np.pi

                #link 2 angle, yellow to blue
                #print("z diff: "+str(centres['Yellow']['z']-centres['Blue']['z']))

                if joint2 > 0.1 and (joint3 > 0.1 or joint3 < -0.1):
                        if (centres['Blue']['z']-centres['Red']['z']) != 0:
                                if (centres['Yellow']['x']-centres['Red']['x'])>0:
                                        joint4 = -np.arctan2((centres['Red']['z']-centres['Blue']['z']),(centres['Red']['x']-centres['Blue']['x']))
                                        print("joint2>0 and red to left of yellow xz")
                                else:
                                        joint4 = -np.arctan2((centres['Red']['z']-centres['Blue']['z']),(centres['Red']['x']-centres['Blue']['x']))
                                        print("joint2>0 and red to right of yellow xz")
                        else:
                                joint4 = 0
                elif joint2 < -0.1 and (joint3 > 0.1 or joint3 < -0.1):

                        if (centres['Blue']['z']-centres['Red']['z']) !=0:
                                if (centres['Yellow']['x']-centres['Red']['x'])>0:
                                        joint4 = np.arctan2((centres['Red']['z']-centres['Blue']['z']),(centres['Red']['x']-centres['Blue']['x']))
                                        print("joint2<0 and red to left of yellow xz")
                                else:
                                        joint4 = -np.arctan2((centres['Red']['z']-centres['Blue']['z']),(centres['Red']['x']-centres['Blue']['x']))
                                        print("joint2<0 and red to right of yellow xz")
                        else:
                                joint4 = 0
                                #print(joint4)
                
                else:
                        joint4 =np.pi/2 + np.arctan2((centres['Red']['z']-centres['Blue']['z']),(centres['Red']['x']-centres['Blue']['x'])) - joint2
                               
                if joint4 > 1.5707:
                        joint4 = joint4 - np.pi
                if joint4 < -1.5707:
                        joint4 = joint4 + np.pi
                joint4 =  round(joint4,2)
                #print(centres)
                red = self.pointToVector(centres['Red'])
                blue = self.pointToVector(centres['Blue'])
                yellow = self.pointToVector(centres['Yellow'])
                green = self.pointToVector(centres['Green'])

                # calculate angle
                hybrid = self.getInnerAngle(green,yellow,blue)
                j4 = self.getInnerAngle(yellow,blue,red)
                
                #adjust for persepctive distortion
                # if j2>0.5:
                #         j3 = j3 - j2*0.2
                # if j3>1:
                #         j2 = j2 - j3*0.6
                # print(np.array([0,j2,j3,j4]))
                ms = self.calpixScale(green, yellow)
                # ms = 1
                mred = ms* red
                mblue = ms * blue
                myellow = ms * yellow
                mgreen = ms * green
                mred = mred - mgreen
                mblue = mblue - mgreen
                myellow = myellow - mgreen
                mgreen = mgreen - mgreen
                self.b = TransformBroadcaster()
                rotation = [0,0,0,1]
                self.b.sendTransform(mred, rotation, Time.now(), 'red', 'link0')
                self.b.sendTransform(mblue, rotation, Time.now(), 'blue', 'link0')
                self.b.sendTransform(myellow, rotation, Time.now(), 'yellow', 'link0')                                                                                                                                                                                                  
                self.b.sendTransform(mgreen, rotation, Time.now(), 'green', 'link0')
                
                # joint2 = np.arctan2(green[1]-blue[1], green[2]-blue[2])
                print(self.calpixScale(yellow, blue))
                # print(self.calpixScale(green, yellow))
                # Z scale = 0.0386                                              
                # X scale = 0.0391
                return np.array([0, joint2, joint3, j4])

        def calpixScale(self, base, p1):
                spacing = 4
                return spacing/np.linalg.norm(base-p1)
#----------------------------------------------------------------------------------------------------------       


        def anglesVis2(self, centres):
                
                red = self.pointToVector(centres['Red'])
                blue = self.pointToVector(centres['Blue'])
                yellow = self.pointToVector(centres['Yellow'])
                green = self.pointToVector(centres['Green'])

                j3 = self.getInnerAngle(green,yellow,blue)
                j4 = self.getInnerAngle(yellow,blue,red)
                self.xyDist(centres['Yellow'], centres['Blue'])
                 #check to see if the blue joint is in the 0 position
                if (self.xyDist(centres['Yellow'], centres['Blue'])<4 ):
                        #check to see if the red joint is also in 0 position
                        if (self.xyDist(centres['Yellow'], centres['Red'])<4):
                                joint1 = 0
                        else:
                                #red joint is usable to calculate the angle
                                joint1 = - np.arctan2(centres['Yellow']['x']-centres['Red']['x'], centres['Yellow']['y']-centres['Red']['y'])
                else:
                        #blue joint is useable to calculate rotation
                        joint1 = - np.arctan2(centres['Yellow']['x']-centres['Blue']['x'], centres['Yellow']['y']-centres['Blue']['y'])

                # if joint1 > 0:
                #         if(self.xDif(centres['Blue'], centres['Yellow'])>0):
                #                 j3 = -j3 
                #                 print("invert j3")
                #         if(self.xDif(centres['Red'], centres['Blue'])<0):
                #                 j4 = -j4
                #                 print("j4 diff pos")
                #         elif self.yDif(centres['Red'], centres['Blue'])<0:
                #                 j4 = -j4
                #                 print("y diff for j4")

       
                
                # if joint1 < 0:
                #         if(self.xDif(centres['Blue'], centres['Yellow'])<0):
                #                 j3 = -j3

                #                 print("invert j3")
              
                #         if(self.xDif(centres['Red'], centres['Blue'])>0):
                #                 if self.yDif(centres['Red'], centres['Blue'])>0:
                #                         j4 = -j4
                #                         print("j4 diff neg,y obscured")
                        
                #         elif self.yDif(centres['Red'], centres['Blue'])>0:
                #                 j4 = -j4
                #                 print("y diff for j4")


#----------------------------------------------------------------------------------------------------------       


                return np.array([round(joint1,2), 0, round(j3,2), round(j4,2)])
        def xDif(self, centre1, centre2):
                 return centre1['x']-centre2['x']
        def yDif(self, centre1, centre2):
                 return centre1['y']-centre2['y']
        def xyDist(self, centre1, centre2):
                x = np.abs(centre1['x']-centre2['x'])
                y = np.abs(centre1['y']-centre2['y'])
                if x>y:
                        return x
                else:
                        return y

        def pointToVector(self, data):
                return np.array([data['x'],data['y'], data['z']])

        def getInnerAngle(self, pt1,pt2,pt3):
                ba = pt1 - pt2
                bc = pt2 - pt3
                cosineAngle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))

                angle = np.arccos(cosineAngle)
                return angle

#----------------------------------------------------------------------------------------------------------       

        # Perform image processing, green base joint not required
        def imProcess(self, image):
    
                green_u = (20,256,20)
                green_l = (0,50,0)
                blue_u = (256,20,20)
                blue_l = (50,0,0)
                red_u = (20,20,256)
                red_l = (0,0,50)
                yellow_u = (0,256,256)
                yellow_l = (0,70,70)
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
                        yzcenter[c]['z'] =  centres['yz'][c]['y']
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
                                        matchCoords[colour][axis] = out
                                else:
                                        matchCoords[colour][axis] = -1
                                if axis == 'z' and matchCoords[colour][axis] >= 0 :
                                        matchCoords[colour][axis] = -matchCoords[colour][axis]

                                        #print("Warning missing data")

                if centres['xz']['Yellow']['y'] == -1:
                        matchCoords["Yellow"]['z'] = yzcenter['Yellow']['y']
                # else:
                #         matchCoords["Yellow"]['z'] = centres['xz']['Yellow']['y']
                   

                if centres['xz']['Blue']['y'] == -1: 
                        matchCoords["Blue"]['z'] = yzcenter['Blue']['y']
                # else:
                #         matchCoords["Blue"]['z'] = centres['xz']['Blue']['y']
                if  matchCoords["Blue"]['y'] == -1: 
                        matchCoords["Blue"]['y'] =  matchCoords["Yellow"]['y']
                if  matchCoords["Blue"]['x'] == -1: 
                        matchCoords["Blue"]['x'] =  matchCoords["Yellow"]['x']
                 
                 
                if centres['xz']['Red']['y'] == -1:
                        matchCoords["Red"]['z'] = yzcenter['Red']['y']
                # else:
                #         matchCoords["Red"]['z'] = centres['xz']['Red']['y']

                if matchCoords["Red"]['x'] == -1:
                        matchCoords["Red"]['x'] = matchCoords["Blue"]['x']
                if matchCoords["Red"]['y'] == -1:
                        matchCoords["Red"]['y'] = matchCoords["Blue"]['y']
                #print(matchCoords)
                return matchCoords
