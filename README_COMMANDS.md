# 🤖 TurtleBot Burger 2.0 — Sequential Terminal Commands Guide
### Terminal-by-Terminal Execution Guide for C++ and Python Packages

---

## 🛑 Step 0: Emergency Stop & Process Cleanup (Run Before Starting)

Run these two commands whenever you need to stop the robot or reset before launching a new mode:

### 1. Emergency Motor Stop (Immediate Hardware Brake)
```bash
python3 -c "import serial, time; s = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.5); s.write(b'STOP\n'); s.flush(); time.sleep(0.1); s.close(); print('Motors STOPPED.')"
```

### 2. Kill All ROS Nodes (Safe for TigerVNC GUI)
```bash
pkill -9 -f "ros2|simple_explorer|arduino_bridge|ydlidar|slam_toolbox|object_detector|rviz2|nav2" || true
```

---

## ⚡ SECTION 1: Pure C++ Package (`custom_turtlebot3_cpp`)
> **Workspace:** `/home/ps/psnh_turtlebot_cpp_ws`  
> **Package:** `custom_bot_cpp`  
> **Key Features:** Zero Python latency, POSIX raw UART, Ceres scan matching, high precision.

---

### 🔨 Phase 1: Build & Environment Setup
Execute this in your terminal before running any C++ nodes:

```bash
cd /home/ps/psnh_turtlebot_cpp_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
```

---

### 🗺️ Mode A: Autonomous Mapping (C++)

#### Option 1: Master Orchestrator (Single Terminal — Recommended)
This single command automatically starts the hardware, SLAM, RViz2 on VNC Display `:1`, and the autonomous frontier explorer in timed sequence:

```bash
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
ros2 run custom_bot_cpp autonomous_mapping_launcher
```

#### Option 2: Terminal-by-Terminal Modular Execution

* **Terminal 1 — Hardware Bringup (URDF + LiDAR + Motor Bridge + ToF + Camera):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp robot_full_launcher
  ```

* **Terminal 2 — SLAM Toolbox (Real-Time Graph Mapping):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp slam_launcher
  ```

* **Terminal 3 — RViz2 Visualizer (Display `:1`):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  export DISPLAY=:1
  export XAUTHORITY=/home/ps/.Xauthority
  rviz2 -d /home/ps/psnh_turtlebot_cpp_ws/install/custom_bot_cpp/share/custom_bot_cpp/config/mapping.rviz
  ```

* **Terminal 4 — Autonomous Frontier Explorer:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp simple_explorer
  ```

---

### 🕹️ Mode B: Manual Mapping with Teleoperation (C++)

* **Terminal 1 — Hardware Bringup:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp robot_full_launcher
  ```

* **Terminal 2 — SLAM Toolbox:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp slam_launcher
  ```

* **Terminal 3 — RViz2 Visualizer (Display `:1`):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  export DISPLAY=:1
  export XAUTHORITY=/home/ps/.Xauthority
  rviz2 -d /home/ps/psnh_turtlebot_cpp_ws/install/custom_bot_cpp/share/custom_bot_cpp/config/mapping.rviz
  ```

* **Terminal 4 — Keyboard Teleoperation (Hold key to move, release to stop):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp teleop_key_hold
  ```

---

### 🧭 Mode C: Autonomous Navigation with Saved Map (C++)

#### Option 1: Master Orchestrator (Single Terminal — Recommended)
```bash
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
ros2 run custom_bot_cpp full_navigation_launcher
```
*(To use a custom map file: add `--map=/home/ps/Desktop/my_map.yaml` at the end)*

#### Option 2: Terminal-by-Terminal Modular Execution

* **Terminal 1 — Hardware Bringup:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp robot_full_launcher
  ```

* **Terminal 2 — Nav2 Navigation Stack (AMCL + Costmaps + Planners):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false map:=/home/ps/psnh_turtlebot_cpp_ws/maps/room_map.yaml params_file:=/home/ps/psnh_turtlebot_cpp_ws/src/custom_bot_nav_cpp/config/nav2_params.yaml
  ```

* **Terminal 3 — RViz2 Navigation GUI (Display `:1`):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  export DISPLAY=:1
  export XAUTHORITY=/home/ps/.Xauthority
  rviz2 -d /home/ps/psnh_turtlebot_cpp_ws/install/custom_bot_cpp/share/custom_bot_cpp/config/nav2.rviz
  ```

* **Terminal 4 (Optional) — 3D Docking Station Visualizer:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp docking_station_visualizer
  ```

* **Terminal 5 (Optional) — Autonomous Waypoint Sequence Follower:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
  ros2 run custom_bot_cpp waypoint_navigator
  ```

---

## 🐍 SECTION 2: Python Package (`nammy_ps_turtlebot3`)
> **Workspace:** `/home/ps/psnh_turtlebot_ws`  
> **Package:** `custom_bot`  
> **Key Features:** Standard ROS 2 launch files, Python dynamic configurations.

---

### 🔨 Phase 1: Build & Environment Setup
Execute this in your terminal before running Python nodes:

```bash
cd /home/ps/psnh_turtlebot_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_ws/install/setup.bash
```

---

### 🗺️ Mode A: Autonomous Mapping (Python)

#### Option 1: Master Launch File (Single Terminal — Recommended)
```bash
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_ws/install/setup.bash
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
ros2 launch custom_bot autonomous_mapping.launch.py
```

#### Option 2: Terminal-by-Terminal Modular Execution

* **Terminal 1 — Hardware Bringup (LiDAR + Motor Bridge + ToF + Camera):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  ros2 launch custom_bot robot_full.launch.py
  ```

* **Terminal 2 — SLAM Toolbox & RViz2 (Display `:1`):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  export DISPLAY=:1
  export XAUTHORITY=/home/ps/.Xauthority
  ros2 launch custom_bot mapping.launch.py
  ```

* **Terminal 3 — Autonomous Frontier Explorer:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  ros2 run custom_bot simple_explorer.py
  ```

---

### 🕹️ Mode B: Manual Mapping with Teleoperation (Python)

* **Terminal 1 — Hardware Bringup:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  ros2 launch custom_bot robot_full.launch.py
  ```

* **Terminal 2 — SLAM Toolbox & RViz2 (Display `:1`):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  export DISPLAY=:1
  export XAUTHORITY=/home/ps/.Xauthority
  ros2 launch custom_bot mapping.launch.py
  ```

* **Terminal 3 — Keyboard Teleoperation (Hold key to move, release to stop):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  ros2 run custom_bot teleop_key_hold.py
  ```

---

### 🧭 Mode C: Autonomous Navigation with Saved Map (Python)

#### Option 1: Master Launch File (Single Terminal — Recommended)
```bash
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_ws/install/setup.bash
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
ros2 launch custom_bot full_navigation.launch.py
```

#### Option 2: Terminal-by-Terminal Modular Execution

* **Terminal 1 — Hardware Bringup:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  ros2 launch custom_bot robot_full.launch.py
  ```

* **Terminal 2 — Nav2 Navigation Stack with Map:**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  ros2 launch custom_bot_nav custom_navigation.launch.py map:=/home/ps/psnh_turtlebot_ws/maps/room_map.yaml
  ```

* **Terminal 3 — RViz2 Navigation GUI (Display `:1`):**
  ```bash
  source /opt/ros/humble/setup.bash
  source /home/ps/psnh_turtlebot_ws/install/setup.bash
  export DISPLAY=:1
  export XAUTHORITY=/home/ps/.Xauthority
  rviz2 -d /home/ps/psnh_turtlebot_ws/src/custom_bot/config/mapping.rviz
  ```

---

## 💾 SECTION 3: Map Saving & Image Conversion

Open a new terminal while SLAM mapping is actively running:

### 1. Save Map to Desktop
```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f /home/ps/Desktop/my_map
```
*(Creates `/home/ps/Desktop/my_map.yaml` and `/home/ps/Desktop/my_map.pgm`)*

### 2. Convert Saved PGM Map to PNG Image
```bash
python3 -c "from PIL import Image; Image.open('/home/ps/Desktop/my_map.pgm').save('/home/ps/Desktop/my_map.png'); print('Map converted to /home/ps/Desktop/my_map.png')"
```

---

## 📸 SECTION 4: Live RViz2 Desktop Screenshot Capture

Capture the current state of Display `:1` without touching TigerVNC:

```bash
DISPLAY=:1 import -window root /home/ps/Desktop/live_rviz_capture.png
```
