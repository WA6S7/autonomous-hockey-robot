# Autonomous Hockey Robot – Simulation Track  
**ARI3215 Robotics 2 – University of Malta**

## Student 1 – Simulation & Robot Modelling

---

## Role Overview

**Student 1** is responsible for the **simulation environment and robot modelling**.  
This includes designing the robot structure, defining sensors, setting up the Gazebo world, and ensuring correct TF frames and ROS 2 integration.

The work focuses on providing a **stable, realistic simulation** that other team members can build upon for perception and AI behaviour.

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

Location: hockey_robot_description/launch/bridge.launch.py



