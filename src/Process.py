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
 
        def anglesVis1(self, localCenters):
                
                X = 0
                Y =1
                Z = 2
                centres = self.shiftAndScalePixels(localCenters)
                red = centres['Red']
                blue = centres['Blue']
                yellow = centres['Yellow']
                green = centres['Green']

                joint2 = np.arctan2(centres['Green'][X]-(centres['Blue'][X]),(-centres['Green'][Z]-centres['Yellow'][Z]))
                if joint2 > np.pi/2:
                        joint2 -= np.pi
                else:
                        joint2 += np.pi
                

                joint3 = ( np.pi/2)-(np.arctan2(centres['Blue'][Z]-centres['Yellow'][Z],centres['Yellow'][Y]-centres['Blue'][Y]) )
                

                joint4 = self.getInnerAngle(yellow,blue,red)
                if green[X] > red[X]:
                        joint4 *=-1
                
                self.b = TransformBroadcaster()
                rotation = [0,0,0,1]
                self.b.sendTransform(red, rotation, Time.now(), 'red', 'link0')
                self.b.sendTransform(blue, rotation, Time.now(), 'blue', 'link0')
                self.b.sendTransform(yellow, rotation, Time.now(), 'yellow', 'link0')                                                                                                                                                                                                  
                self.b.sendTransform(green, rotation, Time.now(), 'green', 'link0')

              
                
               
                # # Z scale = 0.0386                                              
                # # X scale = 0.0391
                return np.array([0, joint2, joint3, joint4])

        def calpixScale(self, base, p1,spacing=4):
                # spacing = 4
                return spacing/np.linalg.norm(base-p1)
#----------------------------------------------------------------------------------------------------------
        
        def anglesVis2(self, localCenters):
                X = 0
                Y =1
                Z = 2
                centres = self.shiftAndScalePixels(localCenters)
                red = centres['Red']
                blue = centres['Blue']
                yellow = centres['Yellow']
                green = centres['Green']
                # red = self.pointToVector(centres['Red'])
                # blue = self.pointToVector(centres['Blue'])
                # yellow = self.pointToVector(centres['Yellow'])
                # green = self.pointToVector(centres['Green'])

                j3 = self.getInnerAngle(green,yellow,blue)
                j4 = self.getInnerAngle(yellow,blue,red)
        
                 #check to see if the blue joint is in the 0 position
                if (self.xyDist(centres['Yellow'], centres['Blue'])<0.35): # Prevent tracking off center blue ball
                        #check to see if the red joint is also in 0 position
                        # print(self.xyDist(centres['Yellow'], centres['Red']))
                        if (self.xyDist(centres['Yellow'], centres['Red'])<0.05):
                                joint1 = 0
                                print("here3")
                        else:
                                #red joint is usable to calculate the angle
                                joint1 = np.arctan2(centres['Yellow'][X]-centres['Red'][X], centres['Yellow'][Y]-centres['Red'][Y])-(np.pi/2)
                                if joint1 < -(np.pi): ##Added Fudge Factor
                                        joint1 += 2*np.pi
                                print("here2")
                else:
                        #blue joint is useable to calculate rotation
                        joint1 = np.arctan2(centres['Yellow'][X]-centres['Blue'][X], centres['Yellow'][Y]-centres['Blue'][Y])
                        if joint1 < -(np.pi+ 0.05): ##Added Fudge Factor
                                joint1 += 2*np.pi
                        print("here1")
                
        
        
                if j3<0 and joint1>0:
                        print("j3<")
                        j3 = -j3
                elif self.yDif(centres['Blue'], centres['Yellow'])>0 and joint1>0:
                        print("j3y")
                        j3 = -j3
                elif self.yDif(centres['Blue'], centres['Yellow'])<0 and joint1<0:
                        print("j3-")
                        j3 = -j3
                
        


                if j3<0:
                        if(self.xDif(centres['Red'], centres['Blue'])>0):
                                j4 = -j4
                                print("j4 diff pos")
                        elif self.yDif(centres['Red'], centres['Blue'])<0:
                                j4 = -j4
                                print("y diff for j4")
                else:
                        if(self.xDif(centres['Red'], centres['Blue'])<0):
                                j4 = -j4
                                print("j4 diff pos")
                        elif self.yDif(centres['Red'], centres['Blue'])>0:
                                j4 = -j4
                                print("y diff for j4")

                # joint1 = np.abs(joint1)
                #NOTE: there are two solutions and no way I can tell to pick the "Correct one". So the best approch I see is the 
                # limit one joint to 180 degrees and then figure out the other two angles to produce a valid robot state
                #  
                # getInnerAngle
                # j1 = joint1 # Smaller var name
                # # j1 = np.pi/2
                # print(blue)
                # rot = np.array([
                #         [np.cos(j1),-np.sin(j1),0],
                #         [np.sin(j1),np.cos(j1),0],
                #         [0,0,1]
                # ])
                # rotblue = np.dot(blue,rot)
                # # print(rotblue)
                # if rotblue[Y] > 0: # Does not work  Atempted to lock joint 1 and figure out 3 and 4
                #         j3 *= -1
        
                return np.array([joint1, 0, j3, j4])
        


#----------------------------------------------------------------------------------------------------------       


                
        def xDif(self, centre1, centre2):
                 return centre1[0]-centre2[0]
        def yDif(self, centre1, centre2):
                 return centre1[1]-centre2[1]
        def xyDist(self, centre1, centre2):
                x = np.abs(centre1[0]-centre2[0])
                y = np.abs(centre1[1]-centre2[1])
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
                                        # out = cord2
                                else:
                                        out = np.max([cord1, cord2])
                                if out > 0:
                                        matchCoords[colour][axis] = out
                                else:
                                        matchCoords[colour][axis] = -1
                                        #print("Warning missing data")

                if centres['xz']['Yellow']['y'] == -1:
                        matchCoords["Yellow"]['z'] = -yzcenter['Yellow']['y']
                # else:
                #         matchCoords["Yellow"]['z'] = centres['xz']['Yellow']['y']
                   

                if centres['xz']['Blue']['y'] == -1: 
                        matchCoords["Blue"]['z'] = -yzcenter['Blue']['y']
                # else:
                #         matchCoords["Blue"]['z'] = centres['xz']['Blue']['y']
                if  matchCoords["Blue"]['y'] == -1: 
                        matchCoords["Blue"]['y'] =  matchCoords["Yellow"]['y']
                if  matchCoords["Blue"]['x'] == -1: 
                        matchCoords["Blue"]['x'] =  matchCoords["Yellow"]['x']
                 
                 
                if centres['xz']['Red']['y'] == -1:
                        matchCoords["Red"]['z'] = -yzcenter['Red']['y']
                # else:
                #         matchCoords["Red"]['z'] = centres['xz']['Red']['y']

                if matchCoords["Red"]['x'] == -1:
                        matchCoords["Red"]['x'] = matchCoords["Blue"]['x']
                if matchCoords["Red"]['y'] == -1:
                        matchCoords["Red"]['y'] = matchCoords["Blue"]['y']

                
                # #print(matchCoords)
                return matchCoords

        def shiftAndScalePixels(self, centers):
                out = {}

                for c in centers:
                        out[c] = self.pointToVector(centers[c])
                # print(out)
                # link1dist = np.linalg.norm(out["Green"]-out["Yellow"])
                # zscale = self.calpixScale(out["Green"],out["Yellow"])
                # xyscale = self.calpixScale(out["Blue"],out["Yellow"],spacing=3.2)
                # xyscale = 3.2/(out["Blue"]-out["Yellow"])
                offset = out['Green']
                for c in out:
                        out[c] = (out[c] - offset)* 0.0386
                        # out
                        out[c][2] *= -1
                out["Yellow"][1] = out["Green"][1] #Force yellow to be inline with green
                out["Yellow"][0] = out["Green"][0]
                out["Yellow"][2] = 4
                print(out)
                return out

