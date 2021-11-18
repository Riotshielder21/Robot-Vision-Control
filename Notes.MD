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