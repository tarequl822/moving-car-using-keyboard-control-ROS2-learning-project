# ROS 2 Jazzy Remote-Controlled Vehicle Learning Project

A complete, beginner-friendly robotics project demonstrating how **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic (gz-sim 8)** work together.

This project implements a full remote-control wheeled vehicle simulator with a Python-based Ground Station, safety watchdogs, custom URDF/Xacro robot description, Gazebo physics simulation, and real-time bidirectional telemetry bridging.

---

## 1. System Architecture & Data Flow

```mermaid
flowchart LR
    subgraph GroundStation["Ground Station (Operator Surface)"]
        GS["ground_station_node\n(Keyboard Input: W/A/S/D/Space)"]
        RViz["RViz2\n(3D TF & Odometry Display)"]
    end

    subgraph ROS2Graph["ROS 2 Jazzy Middleware"]
        CMD["Topic: /vehicle/cmd_vel\n(geometry_msgs/msg/Twist)"]
        ODOM["Topic: /vehicle/odom\n(nav_msgs/msg/Odometry)"]
        CLK["Topic: /clock\n(rosgraph_msgs/msg/Clock)"]
        TF["Topic: /tf\n(tf2_msgs/msg/TFMessage)"]
        MON["vehicle_command_monitor\n(Watchdog Timer: 1.0s Timeout Alert)"]
    end

    subgraph Bridge["ros_gz_bridge"]
        BR_CMD["ROS /vehicle/cmd_vel <--> GZ cmd_vel"]
        BR_ODOM["GZ odom <--> ROS /vehicle/odom"]
        BR_CLK["GZ clock <--> ROS clock"]
        BR_TF["GZ pose <--> ROS tf"]
    end

    subgraph GazeboSim["Gazebo Harmonic Physics Engine"]
        WORLD["Custom World (learning_world.sdf)\nGround Plane + Obstacle Pillars"]
        DIFF["DiffDrive System Plugin\n(Wheel Kinematics & Physics Torques)"]
        ROBOT["3D Vehicle Model\n(Chassis, 2 Drive Wheels, Caster)"]
    end

    GS -->|publishes Twist (10Hz)| CMD
    CMD -->|monitors & safety watchdog| MON
    CMD -->|bridged to Gazebo| BR_CMD
    BR_CMD -->|feeds setpoints| DIFF
    DIFF -->|drives wheel joints| ROBOT
    ROBOT -->|computes wheel odometry| DIFF
    DIFF -->|publishes gz.msgs.Odometry| BR_ODOM
    BR_ODOM -->|converts to nav_msgs/Odometry| ODOM
    ODOM -->|visualizes trail & pose| RViz
```

### Digital Twin: Current Simulator vs. Future Raspberry Pi 5 ROV

| Component | Simulation (This Project) | Physical Subsea ROV (Raspberry Pi 5) |
| :--- | :--- | :--- |
| **Operator Interface** | Laptop: `ground_station_node.py` (WSL terminal) | Laptop: `ground_station_node` (Joystick / Gamepad) |
| **Tether Network** | Linux loopback / shared memory DDS | Waterproof Ethernet tether running ROS 2 UDP/DDS |
| **Safety Watchdog** | `vehicle_command_monitor.py` alerts if commands stop > 1.0s | Onboard Pi watchdog cuts ESC signals if tether disconnects |
| **Controller** | Gazebo Harmonic `DiffDrive` system plugin | Custom onboard thruster allocation node (6 or 8 thrusters) |
| **Actuators** | Simulated wheel revolute joints in Gazebo | Brushless subsea thrusters driven via PWM/I2C ESCs |
| **Telemetry** | Simulated wheel odometry (`/vehicle/odom`) | Onboard IMU (pitch/roll/yaw) + Depth pressure sensor |

---

## 2. Directory Structure

```text
rov_learning_ws/
├── src/
│   ├── vehicle_control/                   # Control & teleoperation nodes
│   │   ├── vehicle_control/
│   │   │   ├── __init__.py
│   │   │   ├── ground_station_node.py     # Keyboard controller with non-blocking raw TTY
│   │   │   ├── vehicle_command_monitor.py # Safety watchdog subscriber & timeout logger
│   │   │   └── test_command_publisher.py  # Automated test maneuver generator
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── vehicle_description/               # 3D robot, worlds, launch & RViz configs
│       ├── urdf/
│       │   └── vehicle.urdf.xacro         # Custom 3D vehicle with DiffDrive & friction physics
│       ├── worlds/
│       │   └── learning_world.sdf         # 3D driving arena with obstacles and sun lighting
│       ├── launch/
│       │   └── sim.launch.py              # Master launch: Gazebo, spawner, bridge, monitor, RViz
│       ├── rviz/
│       │   └── vehicle.rviz               # Pre-configured RViz2 displays for RobotModel & Odometry
│       ├── CMakeLists.txt
│       └── package.xml
├── build/
├── install/
├── log/
└── README.md
```

---

## 3. Prerequisites & Environment Setup

This project requires **Ubuntu 24.04 LTS** (native or via **WSL2** with WSLg graphical support) running **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic**.

### 3.1 Install Required ROS 2 & Gazebo Packages

Run the following command in your terminal:
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-interfaces \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep
```

### 3.2 Initialize and Resolve Dependencies with rosdep
```bash
sudo rosdep init 2>/dev/null || true
rosdep update
cd /mnt/e/meriner_ros/rov_learning_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 4. Building the Workspace

Always build the workspace after modifying URDF/Xacro models, launch files, or Python nodes:

### 4.1 Standard Build
```bash
# 1. Navigate to the workspace root
cd /mnt/e/meriner_ros/rov_learning_ws

# 2. Source the ROS 2 Jazzy underlay
source /opt/ros/jazzy/setup.bash

# 3. Build with symlink-install (allows instant python & resource edits without rebuilding)
colcon build --symlink-install

# 4. Source the built workspace overlay
source install/setup.bash
```

### 4.2 Selective Build (Faster)
To build only a specific package:
```bash
# Build only the control package
colcon build --packages-select vehicle_control --symlink-install

# Or build only the description & simulation package
colcon build --packages-select vehicle_description --symlink-install
```

### 4.3 Clean Build (Resetting the Environment)
If you encounter caching or path issues:
```bash
cd /mnt/e/meriner_ros/rov_learning_ws
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

---

## 5. Project Run Guidelines & Execution Modes

Choose the run mode that fits your development or testing scenario:

### Mode A: Full Interactive Simulation (Recommended)
This workflow launches the 3D physics environment, vehicle state publisher, ROS-Gazebo bridge, safety watchdog monitor, and an interactive keyboard ground station.

#### Terminal 1 — Launch Gazebo Simulation & Safety Monitor
```bash
cd /mnt/e/meriner_ros/rov_learning_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launches Gazebo Harmonic, robot_state_publisher, entity spawner, ros_gz_bridge, and watchdog
ros2 launch vehicle_description sim.launch.py
```

#### Terminal 2 — Run Keyboard Ground Station (Teleoperation)
```bash
cd /mnt/e/meriner_ros/rov_learning_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run the keyboard ground station node
ros2 run vehicle_control ground_station_node
```
> [!TIP]
> Keep **Terminal 2 focused** while driving so keyboard strokes are captured in real time.

---

### Mode B: Simulation with RViz2 Digital Twin Visualization
To inspect coordinate frames (TF tree), robot model geometry, and live odometry trajectories in RViz2 alongside Gazebo:

#### Terminal 1 — Launch Simulation + RViz2
```bash
cd /mnt/e/meriner_ros/rov_learning_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Pass rviz:=true to start RViz with preloaded vehicle.rviz configuration
ros2 launch vehicle_description sim.launch.py rviz:=true
```

#### Terminal 2 — Run Keyboard Ground Station
```bash
cd /mnt/e/meriner_ros/rov_learning_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run vehicle_control ground_station_node
```

---

### Mode C: Automated Maneuver Testing (No Keyboard Needed)
Use the automated test publisher to run a continuous sequence of maneuvers (Forward $\rightarrow$ Stop $\rightarrow$ Turn Left $\rightarrow$ Stop $\rightarrow$ Reverse $\rightarrow$ Stop $\rightarrow$ Turn Right $\rightarrow$ Stop):

#### Terminal 1 — Launch Gazebo Simulation
```bash
cd /mnt/e/meriner_ros/rov_learning_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch vehicle_description sim.launch.py
```

#### Terminal 2 — Run Automated Maneuver Publisher
```bash
cd /mnt/e/meriner_ros/rov_learning_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run vehicle_control test_command_publisher
```

---

### Mode D: Automated Safety Watchdog Integration Test Script
To run an automated integration test verifying node startup, velocity publication, and safety watchdog timeout alerting without launching the full 3D Gazebo GUI:

```bash
cd /mnt/e/meriner_ros/rov_learning_ws
chmod +x test_stage2.sh
./test_stage2.sh
```

---

### Mode E: Manual Command Line Driving (`ros2 topic pub`)
You can inject velocity commands directly from any terminal to test the robot's physical response:

```bash
source /opt/ros/jazzy/setup.bash

# 1. Drive Forward (0.5 m/s)
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 2. Turn Left (0.8 rad/s)
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.8}}'

# 3. Turn Right (-0.8 rad/s)
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.8}}'

# 4. Reverse (-0.5 m/s)
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# 5. Immediate Emergency Stop (Halt)
ros2 topic pub --once /vehicle/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

---

## 6. Keyboard Driving Controls

When running [ground_station_node.py](file:///e:/meriner_ros/rov_learning_ws/src/vehicle_control/vehicle_control/ground_station_node.py) with its terminal focused:

| Key | Action | Commanded Values | Behavior in Simulation |
| :---: | :--- | :--- | :--- |
| **`W`** | Drive Forward | `linear.x = +0.5 m/s` | Both wheels spin forward |
| **`S`** | Reverse | `linear.x = -0.5 m/s` | Both wheels spin in reverse |
| **`A`** | Turn Left | `angular.z = +1.0 rad/s` | Left wheel reverses, right wheel pushes forward |
| **`D`** | Turn Right | `angular.z = -1.0 rad/s` | Right wheel reverses, left wheel pushes forward |
| **`SPACE`** | Emergency Stop | `linear = 0, angular = 0` | Vehicle immediately halts |
| **`+`** / **`=`** | Speed Up | Increases velocity caps | Vehicle accelerates faster |
| **`-`** / **`_`** | Slow Down | Decreases velocity caps | Fine-grained, precision maneuvering |
| **`Q`** | Safe Exit | Zero velocity + restore TTY | Exits cleanly without corrupting terminal |

---

## 7. Diagnostics, Telemetry & Inspection Commands

Open an additional terminal to inspect real-time system performance, topics, and transformations:

### 7.1 Inspect Active Nodes and Topics
```bash
source /opt/ros/jazzy/setup.bash

# List all active ROS 2 nodes
ros2 node list

# List all active topics
ros2 topic list -t

# Check publisher and subscriber counts on command topic
ros2 topic info /vehicle/cmd_vel --verbose
```

### 7.2 Monitor Live Telemetry Streams
```bash
# Echo live wheel odometry (position, orientation, linear & angular velocities)
ros2 topic echo /vehicle/odom

# Echo live commanded velocity
ros2 topic echo /vehicle/cmd_vel

# Echo coordinate frames
ros2 topic echo /tf
```

### 7.3 Measure Publication Rates and Bandwidth
```bash
# Check publication frequency of ground station commands (nominal: 10 Hz)
ros2 topic hz /vehicle/cmd_vel

# Check publication rate of Gazebo odometry bridge
ros2 topic hz /vehicle/odom

# Check data throughput (bandwidth)
ros2 topic bw /vehicle/odom
```

### 7.4 Watchdog Node & Parameter Inspection
```bash
# Inspect parameters configured on the watchdog monitor
ros2 param list /vehicle_command_monitor

# Get watchdog timeout threshold
ros2 param get /vehicle_command_monitor timeout_seconds

# Dynamically adjust the timeout threshold at runtime (e.g., to 2.5 seconds)
ros2 param set /vehicle_command_monitor timeout_seconds 2.5
```

### 7.5 Inspect TF Transform Tree
```bash
# Echo the dynamic transform between odom and base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint

# Generate a PDF visual graph of the full transform tree (frames.pdf)
ros2 run tf2_tools view_frames
```

---

## 8. Safety Features Implemented

1. **Safety Watchdog Timer**:
   [vehicle_command_monitor.py](file:///e:/meriner_ros/rov_learning_ws/src/vehicle_control/vehicle_control/vehicle_command_monitor.py) measures the elapsed time since the last command:
   $$\Delta t = t_{\text{now}} - t_{\text{last\_command}}$$
   If $\Delta t > 1.0\text{ s}$, a `[TIMEOUT ALERT]` is triggered. In a physical vehicle, this halts motors if the communications link is interrupted.
2. **Ground Station Heartbeat**:
   [ground_station_node.py](file:///e:/meriner_ros/rov_learning_ws/src/vehicle_control/vehicle_control/ground_station_node.py) publishes at a steady 10 Hz rate to prevent starving the watchdog while active.
3. **Idle Auto-Deceleration**:
   If the operator releases the keyboard for $> 0.8\text{ s}$, the ground station node automatically drops setpoints to $0.0\text{ m/s}$.
4. **Terminal Restoration**:
   `termios` raw terminal mode is protected by a Python `finally:` block, ensuring the Linux shell prompt is restored cleanly even on `Ctrl+C`.

---

## 9. Troubleshooting Guide

* **Gazebo window does not appear in WSL:**
  Ensure WSLg display environment variable is set:
  ```bash
  export DISPLAY=:0
  gz sim shapes.sdf
  ```
  If graphics driver hardware acceleration fails in WSL, force software OpenGL rendering:
  ```bash
  export LIBGL_ALWAYS_SOFTWARE=1
  ```

* **Node or package cannot be found by `ros2 run` / `ros2 launch`:**
  Remember to source both the system underlay and your workspace overlay:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /mnt/e/meriner_ros/rov_learning_ws/install/setup.bash
  ```

* **Terminal keystrokes not responding:**
  Ensure your terminal window running `ground_station_node` has active cursor focus. Do not click into the Gazebo window while typing driving commands.

* **Vehicle drifts or doesn't turn sharply:**
  The spherical caster wheel in [vehicle.urdf.xacro](file:///e:/meriner_ros/rov_learning_ws/src/vehicle_description/urdf/vehicle.urdf.xacro) is configured with minimal surface friction (`mu1=0.001`, `mu2=0.001`). If higher ground traction is required, drive wheel friction coefficients (`mu1`, `mu2`) can be adjusted in the `<gazebo reference="left_wheel_link">` tags.

---

## 10. Quick Command Reference Cheat Sheet

| Task | Command |
| :--- | :--- |
| **Install Dependencies** | `rosdep install --from-paths src --ignore-src -r -y` |
| **Build Entire Workspace** | `colcon build --symlink-install` |
| **Build Control Package Only** | `colcon build --packages-select vehicle_control --symlink-install` |
| **Build Description Package Only** | `colcon build --packages-select vehicle_description --symlink-install` |
| **Clean Workspace** | `rm -rf build/ install/ log/ && colcon build --symlink-install` |
| **Source Environment** | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| **Launch Simulation** | `ros2 launch vehicle_description sim.launch.py` |
| **Launch Simulation + RViz** | `ros2 launch vehicle_description sim.launch.py rviz:=true` |
| **Run Keyboard Teleop** | `ros2 run vehicle_control ground_station_node` |
| **Run Automated Maneuver Test** | `ros2 run vehicle_control test_command_publisher` |
| **Run Watchdog Script** | `./test_stage2.sh` |
| **Inspect Odometry Telemetry** | `ros2 topic echo /vehicle/odom` |
| **Check Command Rate (Hz)** | `ros2 topic hz /vehicle/cmd_vel` |
| **List Active Nodes** | `ros2 node list` |

