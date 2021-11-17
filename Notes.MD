## Notes:

1. We need to refactor this git repo. No ned to publish the whole catkin workspace
2. asdf




```bash

roslaunch ivr_assignment spawn.launch
rostopic pub -1 /robot/joint1_position_controller/command std_msgs/Float64 "data: 1.0"
```