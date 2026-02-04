# Autonomous Hockey Robot – Simulation Track  
**ARI3215 Robotics 2 – University of Malta**

Student 1 – Simulation & Robot Modelling – Nikolina Filipov Pajic\
Student 2 – Puck Detection & Tracking – Jeremy Galea\
Student 3 – AI Control & Behaviour – Liam Jake Vella

---

## Role Overview

**Student 1** is responsible for the **simulation environment and robot modelling**.  
This includes designing the robot structure, defining sensors, setting up the Gazebo world, and ensuring correct TF frames and ROS 2 integration.

**Student 2** is responsible for the **implementation of the puck detection and tracking**.  
This includes writing a puck detection script, to be able to detect the puck using a LiDAR sensor. Once this is implemented successfully, a puck tracking script is implemented to be able to track the puck movement during execution. 

The work focuses on having a reliable and accurate puck detection and tracking system.

**Student 3** is responsible for the **design and implementation of the AI Control & Behaviour**.  
This includes the Finite State Machine (FSM) and the autonomous strategy that governs the robot's decision-making process.

The work focuses on enabling the robot to autonomously navigate the arena, locate the puck, and attempt to score while avoiding obstacles.

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

## How to Run

### Prerequisites
Before running the simulation, ensures the workspace is built and sourced correctly.
```bash
colcon build
source /opt/ros/jazzy/setup.bash
source install/setup.bash 
```

### Option 1: Run Everything in One Terminal (Recommended)
This singular bringup file launches the entire simulation including robots, the arena, and the ROS–Gazebo bridge.
```bash
source install/setup.bash
ros2 launch hockey_robot_description hockey_bringup.launch.py
```

### Option 2: Run in Separate Terminals
For manual control or debugging individual components:

1. **Launch Robots and World:**
   ```bash
   source install/setup.bash
   ros2 launch hockey_robot_description spawn_robots.launch.py 
   ```

2. **ROS–Gazebo Bridge:**
   ```bash
   source install/setup.bash 
   ros2 launch hockey_robot_description bridge.launch.py
   ```

3. **Puck Detection & Tracking:**
   *Robot 1:*
   ```bash
   source install/setup.bash
   ros2 run hockey_perception lidar_puck_detector --ros-args \
     -r __node:=lidar_puck_detector_r1 \
     -p scan_topic:=/robot1/scan \
     -p puck_pose_topic:=/robot1/puck_pose_lidar
   ```
   *Robot 2:*
   ```bash
   source install/setup.bash
   ros2 run hockey_perception lidar_puck_detector --ros-args \
     -r __node:=lidar_puck_detector_r2 \
     -p scan_topic:=/robot2/scan \
     -p puck_pose_topic:=/robot2/puck_pose_lidar
   ```

### Monitoring & Debugging (Optional)
- **Start RViz:** `rviz2`
-- In RViz, set the Fixed Frame to 'robot1/base_link' or 'robot2/base_link' and add TF, LaserScan, and RobotModel
displays.
- **Verify Puck Detection Hz:** `ros2 topic hz /robot1/puck_pose_lidar`
- **View Puck Pose Data:** `ros2 topic echo /robot1/puck_pose_lidar`

---

## Student 1 Responsibilities

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

## Student 2 Responsibilities

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

## Student 3 Responsibilities

### 1. Finite State Machine (FSM)
- Implemented a robust **Finite State Machine (FSM)** to handle different operational modes:
  - **SEARCH**: If the puck is not detected for over 1.0 second, the robot rotates at a constant angular velocity (0.8 rad/s) to scan the environment.
  - **APPROACH**: Once the puck is found via the `puck_pose_lidar` topic, a proportional (P) controller aligns the robot's heading and drives it toward the puck (maintaining a minimum speed of 0.3 m/s).
  - **AVOID**: This state has the highest priority; if an object is detected within 0.35 m (that is not the puck), the robot executes a recovery maneuver by backing up and turning.



---

### 2. Control Logic and Navigation
- Developed the `simple_fsm.py` node to process perception data and generate velocity commands.
- **Obstacle Discrimination**: The logic compares LiDAR returns against the known puck position to prevent the robot from misidentifying the puck as an obstacle, ensuring smooth interaction.
- **Velocity Commands**: Decisions are translated into `geometry_msgs/Twist` commands sent to the `/cmd_vel` topic.
- **Namespacing**: The node handles `robot_name` and `puck_topic` parameters, allowing the same control logic to be deployed across multiple robots.

---

---

## Known Issues
- **Puck Shape Modification**: Due to LiDAR detection limitations with thin cylinders, the hockey puck's visual model was changed to a **ball** to ensure more reliable tracking and detection.
- **Physics Interaction**: Due to simulation constraints and time limitations, standard physics interactions sometimes fail to move the ball effectively. The `puck_kicker.py` utility was developed as an attempt to apply direct force bursts to the puck.

---

## AI Concepts Demonstrated
The project demonstrates core robotics AI concepts including:
- **Perception:** Real-time LiDAR and IMU data processing.
- **Reasoning:** State-based behaviour via a Finite State Machine (FSM).
- **Action:** Autonomous navigation and obstacle avoidance using velocity commands.

---

## Academic Integrity
All work is original and developed for **ARI3215 Robotics 2**. Standard ROS 2 and Gazebo libraries are utilized in accordance with their official documentation and licensing.
