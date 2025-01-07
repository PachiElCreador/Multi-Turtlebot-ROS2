# Useful Commands and Resources

This document contains a collection of commands, configurations, and relevant notes organized by topics.



## Access and Configuration

### Open Folder of Ros packges
```bash
sudo nautilus /opt/ros/humble/share/
```


## Simulation

```bash
cd turtlebot3_ws/
colcon build --symlink-install

ros2 launch turtlebot3_gazebo multi_tb.launch.py
```

#### Teleoperation Mode
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb1/cmd_vel
```
```bash
ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb2/cmd_vel
```

#### Autonomous driving
  ```bash
  ros2 run turtlebot3_gazebo turtlebot3_drive --ros-args -r /cmd_vel:=/tb1/cmd_vel
  ```


### Example Simulation from Robotis
```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True namespace:=/tb1
```
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True namespace:=/tb2
```

Run Teleoperation Mode
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```
Save Map
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

---

## Terminate Residual Processes

### Kill All Relevant Processes
```bash
pkill -f gazebo
pkill -f rviz
pkill -f ros2
```

### List Active Processes
```bash
ps aux | grep gazebo
ps aux | grep ros2
ps aux | grep rviz
```

---

## ROS 2 Utilities

### List Active Nodes
```bash
ros2 node list
```

### List Available Topics
```bash
ros2 topic list
```

### Generate Environment PDF
```bash
ros2 run tf2_tools view_frames
```

---

## Additional Notes



## Miscellaneous Commands

- **CTRL+C:** Terminate processes
