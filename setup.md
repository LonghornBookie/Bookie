Set-Up Segway
```
Install necessary packages: https://github.com/Living-With-Robots-Lab/bwi_ros2_minimal
```
```
cd ~/bwi_ros2_final/src
git clone https://github.com/Living-With-Robots-Lab/ros2segway.git
git clone https://github.com/utexas-bwi/segway_msgs.git
```
Running
```
cd ~/bwi_ros2_final
colcon build
source install/setup.bash
```
Ros2 Run Commands
```
ros2 launch bwi_launch segbot_v2.launch.py
```
```
ros2 launch azure_kinect_ros_driver driver.launch.py
```
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
To run nav_goals, use RVIZ prompt launched by ros2 launch bwi_launch segbot_v2.launch.py
Do not use azure_kinect, untested.
```
mkdir -p ~/bwi_ros2_final/src/temporary/nav_goals/src
cd ~/bwi_ros2_final/src/temporary/nav_goals/src
# Place send_goal.cpp in this folder
```