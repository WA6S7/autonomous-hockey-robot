# Autonomous Hockey Robot – Simulation Track  
**ARI3215 Robotics 2 – University of Malta**

## Student 1 – Simulation & Robot Modelling
## Student 2 – Puck Detection & Tracking

---

## Role Overview

**Student 1** is responsible for the **simulation environment and robot modelling**.  
This includes designing the robot structure, defining sensors, setting up the Gazebo world, and ensuring correct TF frames and ROS 2 integration.

The work focuses on providing a **stable, realistic simulation** that other team members can build upon for perception and AI behaviour.

---

## Required Packages & Dependencies

sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-tf2-tools \
  ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs \
  ros-jazzy-nav-msgs \
  python3-colcon-common-extensions \
  python3-rosdep \
  gz-sim



---

## Responsibilities

### 1. Robot Modelling (URDF/Xacro)
- Designed the hockey robot using **URDF with Xacro**
- Implemented:
  - `base_link`
  - Wheels and casters
  - LiDAR sensor
  - IMU sensor
- Used **prefixing** (`robot1_`, `robot2_`) to support **multi-robot simulation**
- Ensured correct:
  - Joint definitions
  - Inertial parameters
  - Visual and collision geometry

Location: hockey_robot_description/urdf/hockey_robot.urdf.xacro


---

### 2. Simulation World
- Built a **custom hockey arena** using Gazebo SDF
- Arena includes:
  - Walls
  - Goals
  - Ground plane
- Designed to support puck interaction and autonomous navigation

hockey_world/worlds/hockey_arena.sdf


---

### 3. Launch Files
- Created launch files to:
  - Start Gazebo
  - Spawn multiple robots
  - Load robot descriptions
- Robots are spawned with:
  - Separate namespaces
  - Independent TF trees

Location: hockey_robot_description/launch/spawn_robots.launch.py


---

### 4. TF Frames & State Publishing
- Integrated `robot_state_publisher`
- Verified correct TF tree for:
  - `base_link`
  - `lidar_link`
  - `imu_link`
- Ensured compatibility with RViz and downstream perception nodes

---

### 5. ROS–Gazebo Bridging
- Configured ROS 2 ↔ Gazebo bridges for:
  - `/scan` (LiDAR)
  - `/imu`
  - `/cmd_vel`
- Supports real-time interaction between control nodes and simulation

---

**Student 2** is responsible for the **implementation of the puck detection and tracking**.  
This includes writing a puck detection script, to be able to detect the puck using a LiDAR sensor. Once this is implemented successfully, a puck tracking script is implemented to be able to track the puck movement during execution. 

The work focuses on having a reliable and accurate puck detection and tracking system.

## Responsibilities

### 1. Puck Detection and Tracking Script
- Integrated the `lidar_puck_detector.py`
  - Detects and tracks the puck

Location: hockey_perception/hockey_perception/lidar_puck_detector.py


---

### 2. Bringup File
- Created a singular bringup file to avoid a multiple terminal system.
- **Important:** Due to hardware restrictions and the software being used, sometime not all the files manage to launch.

Location: hockey_robot_description/launch/hockey_bringup.launch.py


---

### Edited Previous Code
- Made minor changes in:
  - `bridge.launch.py`: changes in Laserscan arguments.
  - `spawn_robots.launch.py`: changes in robots' spawning orientation.
  - `hockey_robot.urdf.xacro`: Raised the base slightly off the ground, fixed rear-wheel orientation, and minor changes to the LiDAR.
  - `hockey_arena.sdf`: added physics, plugins, and changed the **puck** to a **ball**.


---
