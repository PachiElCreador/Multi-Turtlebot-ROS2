# TurtleBot3 Multi-Robot SLAM Project

## Project Description

This repository contains the files necessary to configure and operate a multi-robot simulation using TurtleBot3 in ROS 2 Humble and Gazebo. The main objective is to implement a SLAM system for multiple robots in a simulated environment. The project includes:

- **Simulation of TurtleBot3 robots** in a Gazebo world.
- **Configuration of multiple namespaces** for each robot.
- **Launch of state publisher and robot spawn nodes**.
- Attempts to integrate **Cartographer** for simultaneous localization and mapping.

## Workspace Setup

### Prerequisites
- **ROS 2 Humble** installed.
- **Gazebo 11** installed.
- TurtleBot3 dependencies:
  ```bash
  sudo apt install ros-humble-turtlebot3-msgs ros-humble-turtlebot3-gazebo ros-humble-cartographer
  ```

### Repository Cloning

```bash
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src
git clone <REPOSITORY_URL>
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

## Key Files

### 1. **`turtlebot3_house.launch.py`**
This file launches the simulation in Gazebo and spawns multiple TurtleBot3 robots.

**Purpose and Configuration:**
- Implements namespaces to separate robots.
- Uses `robot_state_publisher` and `spawn_entity.py` to initialize each robot.

### 2. **`slam_cart.launch.py`**
A modified file based on Cartographer to attempt SLAM with a specific namespace.

**Issues Encountered:**
- `imu_link` was not correctly registered in `tf`, causing transformation errors.
- The `map` frame was not generated correctly.

### 3. **`occupancy_grid.launch.py`**
Node that generates a map based on Cartographer data.

**Purpose:**
- Attempts to remap topics to the corresponding namespace.
- Ensures the map is published within the desired namespace.

### 4. **`src`**
This directory contains the original and modified files for the TurtleBot3 simulation.

```plaintext
src/
|- turtlebot3_simulations/
   |- turtlebot3_gazebo/
      |- launch/
         |- turtlebot3_house.launch.py
         |- slam_cart.launch.py
         |- occupancy_grid.launch.py
      |- urdf/
         |- turtlebot3_burger.urdf
```

## Results Achieved

### 1. Multi-Robot Simulation
- **Status:** Completed.
- **Details:** Successfully spawned two robots (`tb1` and `tb2`) in Gazebo with independent namespace configurations.
- **Command:**
  ```bash
  ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
  ```

### 2. SLAM Integration with Cartographer
- **Status:** Partial.
- **Details:** While Cartographer was launched using `slam_cart.launch.py`, a map could not be displayed in `rviz` due to transformation issues (`imu_link`).

### 3. Tested Commands
- **Move robots:**
  ```bash
  ros2 run turtlebot3_teleop teleop_keyboard --ros-args -r /cmd_vel:=/tb1/cmd_vel
  ```
- **Verify transformations:**
  ```bash
  ros2 run tf2_tools view_frames
  ```

## Next Steps 
### Prioritized

## 1. Resolve issues with `imu_link` in Cartographer
- **Difficulty:** Medium (1-4 weeks)  
- **Relevance:** High

## 2. Validate `tf` transformations between `base_link`, `odom`, and `map`
- **Difficulty:** Medium  
- **Relevance:** High 

## 3. Complete mapping in `rviz` and integrate navigation
- **Difficulty:** Medium 
- **Relevance:** High 

## 4. Implement Elena and Jarouss' MPC together with navigation
- **Difficulty:** Probably Medium
- **Relevance:** High 

## 5. Move from simulation to physical implementation with 2+ TurtleBots
- **Difficulty:** Low-Medium  
- **Relevance:** Medium-High

---

### Optional Tasks

## 6. Integrate all necessary components into a single launch file
- **Difficulty:** Low-Medium
- **Relevance:** Medium (would simplify operations and enhance project reproducibility)  

## 7. Port the project to C++ for scalability
- **Difficulty:** High
- **Relevance:** Low (not critical for immediate success but useful for larger projects)  


## Contribution
For any questions or suggestions you can contact oscarlc10@outlook.com

## References
- [TurtleBot3 Official Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/#overview)
- [Cartographer ROS Integration](https://google-cartographer-ros.readthedocs.io/en/latest/)
- [ROS 2 Cartographer](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html)



