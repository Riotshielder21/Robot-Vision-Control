## Notes:

[x]. We need to refactor this git repo. No ned to publish the whole catkin workspace
[]. Change how we are doing the ros messages to move away from json
[]. Change back Simulate speed to the default. (Remove Dev preformance mode) worldfile





```bash
rosrun rviz rviz -d ./rviz/robot-labels.rviz
rosrun ivr_assignment control.py
roslaunch ivr_assignment spawn.launch
rostopic pub -1 /robot/joint1_position_controller/command std_msgs/Float64 "data: 1.0"
rostopic pub -1 /robot/joint2_position_controller/command std_msgs/Float64 "data: 1.0"
rostopic pub -1 /robot/joint3_position_controller/command std_msgs/Float64 "data: 1.0"
```
Notes On better cicle detect but it is unstable :(
im_copy = Bframe_gray.copy()
        # # Bframe_gray
        # # circles = cv2.HoughCircles(im_copy,cv2.HOUGH_GRADIENT,1,20,
        # #                     param1=50,param2=30,minRadius=0,maxRadius=100)
        # # print(circles)
        # # circles = np.uint16(np.around(circles))
        # # for i in circles[0,:]:
        # #         # draw the outer circle
        # #         cv2.circle(im_copy,(i[0],i[1]),i[2],(0,255,0),2)
        # #         # draw the center of the circle
        # #         cv2.circle(im_copy,(i[0],i[1]),2,(0,0,255),3)
        # cv2.drawContours(im_copy, Bcontour, -1, (255, 255, 255), 2, cv2.LINE_AA)
        # circles = cv2.HoughCircles(im_copy,cv2.HOUGH_GRADIENT,1,100,
        #                             param1=50,param2=3,minRadius=0,maxRadius=40)
        # circles = np.uint16(np.around(circles))
        # for w in circles[0,:]:
        #         # draw the outer circle
        #         cv2.circle(image,(w[0],w[1]),w[2],(0,255,0),1)
        #         # draw the center of the circle
        #         cv2.circle(image,(w[0],w[1]),2,(0,0,255),1)
        # print (circles)
        # # cv2.circle(im_copy,(int(Bx),int(By)),int(Bradius),(0,255,0),2)
        # # cv2.drawContours(im_copy, Bcontour, -1, (255, 255, 255), 2, cv2.LINE_AA)