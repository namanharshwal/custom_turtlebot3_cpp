# 🤖 Custom TurtleBot Burger 2.0 — Master Command & Operations Manual
### Complete Operational Reference for Both Workspaces (C++ & Python)

---

## 👨‍💻 Authorship & Repository Metadata

* **Author, Creator & Lead Maintainer:** **Naman Harshwal** ([@namanharshwal](https://github.com/namanharshwal) | `namanharshwal@gmail.com`)
* **Project Owner / Supervisor:** **Pradeep Singh Chauhan** (`ps@robot.com`)
* **Platform:** Raspberry Pi 4B (2GB RAM) + Arduino UNO R3 + Bosch BNO055 IMU + YDLidar X2 + VL53L0X ToF + OV5647 CSI Camera
* **OS / Framework:** Ubuntu 22.04 LTS / ROS 2 Humble Hawksbill
* **GUI / VNC Display:** Display `:1` (TigerVNC Server running at resolution `1920x1080`)
* **Repositories:**
  * **C++ Package (Recommended / Golden Standard):** [custom_turtlebot3_cpp](https://github.com/namanharshwal/custom_turtlebot3_cpp.git) — Path: `/home/ps/psnh_turtlebot_cpp_ws`
  * **Python Package:** [nammy_ps_turtlebot3](https://github.com/namanharshwal/nammy_ps_turtlebot3.git) — Path: `/home/ps/psnh_turtlebot_ws`

---

## ⚠️ Critical Ground Rules & Safety Constraints

> [!CAUTION]
> **NEVER KILL OR RESTART TIGERVNC (DISPLAY `:1`):**
> * Never run `vncserver -kill :1`, `systemctl restart vnc*`, `killall Xvnc`, or delete `/tmp/.X1-lock` / `/tmp/.X11-unix/X1`.
> * The live GUI is constantly monitored via VNC on `:1`. Always launch RViz2 and graphical tools with:
>   ```bash
>   export DISPLAY=:1
>   export XAUTHORITY=/home/ps/.Xauthority
>   ```

> [!WARNING]
> **SINGLE ACTIVE CONTROLLER RULE:**
> * Never run both C++ and Python packages simultaneously. Both packages access the same hardware serial port (`/dev/ttyAMA0`), LiDAR (`/dev/ydlidar`), and I2C bus (`/dev/i2c-1`). Always stop all running nodes before switching stacks.

---

## 🛑 Step 0: Emergency Stop & Process Cleanup

Before starting any new mapping or navigation session, run this clean-up routine to release serial ports and stop motor actuation.

### 1. Stop Robot Motors Immediately (Emergency Brake)
```bash
python3 -c "import serial, time; s = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.5); s.write(b'STOP\n'); s.flush(); time.sleep(0.1); s.close(); print('Motors STOPPED successfully.')"
```

### 2. Kill All Active ROS 2 Robot Processes (Safe for TigerVNC)
```bash
pkill -9 -f "ros2|simple_explorer|arduino_bridge|ydlidar|slam_toolbox|object_detector|rviz2|nav2" || true
```

### 3. Verify Hardware Devices Are Free
```bash
fuser /dev/ttyAMA0 || echo "Serial port /dev/ttyAMA0 is FREE"
fuser /dev/ydlidar || echo "LiDAR port /dev/ydlidar is FREE"
```

---

## 🚀 SECTION 1: Pure C++17 Package (`custom_turtlebot3_cpp`)

> **Workspace Path:** `/home/ps/psnh_turtlebot_cpp_ws`  
> **Key Advantage:** Zero Python runtime overhead, POSIX raw termios non-blocking serial (0.1ms latency), Ceres scan-matching graph SLAM, no line-buffering lag, crisp distortion-free 90° maps.

### 🛠️ 1. Build and Sourcing
```bash
cd /home/ps/psnh_turtlebot_cpp_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash
```

---

### 🗺️ 2. Autonomous Mapping (Golden Standard - One Command)
Orchestrates hardware bringup, YDLidar, IMU-fused Arduino bridge, VL53L0X ToF, SLAM Toolbox, RViz2 on Display `:1`, and Simple Explorer with frontier pursuit and stall recovery.

```bash
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && export DISPLAY=:1 && export XAUTHORITY=/home/ps/.Xauthority && exec ros2 run custom_bot_cpp autonomous_mapping_launcher"
```

* **Staging sequence:**
  * `t = 0s`: `robot_state_publisher` + `ydlidar_ros2_driver_node`
  * `t = 2s`: `arduino_bridge` (ticks: 412L/440R, crawl PWM: 92L/98R) + `vl53l0x_node`
  * `t = 3s`: `object_detector_node` (Camera & YOLOv4-tiny)
  * `t = 5s`: `slam_toolbox` (Ceres scan matching)
  * `t = 7s`: `rviz2` on Display `:1`
  * `t = 10s`: `simple_explorer` (Yamauchi frontier detection & anti-stall)

---

### 🧭 3. Autonomous Nav2 Navigation (Using Saved Map)
Launches full hardware stack, Nav2 stack with AMCL particle filter localization, costmaps, planner/controller servers, and RViz2 navigation UI on Display `:1`.

```bash
# With default room map:
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && export DISPLAY=:1 && export XAUTHORITY=/home/ps/.Xauthority && exec ros2 run custom_bot_cpp full_navigation_launcher"

# Or with a custom saved map:
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && export DISPLAY=:1 && export XAUTHORITY=/home/ps/.Xauthority && exec ros2 run custom_bot_cpp full_navigation_launcher --map=/home/ps/Desktop/my_map.yaml"
```

---

### 🕹️ 4. Manual SLAM Mapping (Keyboard Driven)

#### Terminal 1: Hardware Stack Bringup
```bash
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && exec ros2 run custom_bot_cpp robot_full_launcher"
```

#### Terminal 2: SLAM Toolbox Launcher
```bash
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && exec ros2 run custom_bot_cpp slam_launcher"
```

#### Terminal 3: RViz2 Visualization (Display `:1`)
```bash
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && export DISPLAY=:1 && export XAUTHORITY=/home/ps/.Xauthority && exec rviz2 -d /home/ps/psnh_turtlebot_cpp_ws/install/custom_bot_cpp/share/custom_bot_cpp/config/mapping.rviz"
```

#### Terminal 4: Smooth Keyboard Teleoperation (Deadman Hold-to-Move)
```bash
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_cpp_ws/install/setup.bash && exec ros2 run custom_bot_cpp teleop_key_hold"
```

---

### ⚡ 5. Additional C++ Utilities & Nodes

#### 3D Autonomous Docking Station Visualizer
Publishes charging dock geometry and chevrons on `/docking_station/marker`:
```bash
ros2 run custom_bot_cpp docking_station_visualizer --ros-args -p dock_x:=0.0 -p dock_y:=0.0 -p dock_yaw:=0.0
```

#### Autonomous Waypoint Sequence Follower (Nav2 Action Client)
Commands the robot through pre-programmed inspection waypoints:
```bash
ros2 run custom_bot_cpp waypoint_navigator
```

#### Standalone VL53L0X C++ Driver (I2C 0x29)
```bash
ros2 run vl53l0x_sensor_cpp vl53l0x_node --ros-args -p i2c_bus:=1 -p i2c_address:=41 -p max_range:=0.45
```

---

## 🐍 SECTION 2: Python Package (`nammy_ps_turtlebot3`)

> **Workspace Path:** `/home/ps/psnh_turtlebot_ws`  
> **Key Features:** Standard ROS 2 launch system, dynamic parameters, Python ROS nodes.

### 🛠️ 1. Build and Sourcing
```bash
cd /home/ps/psnh_turtlebot_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source /home/ps/psnh_turtlebot_ws/install/setup.bash
```

---

### 🗺️ 2. Autonomous Mapping (One Command)
```bash
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_ws/install/setup.bash && export DISPLAY=:1 && export XAUTHORITY=/home/ps/.Xauthority && exec ros2 launch custom_bot autonomous_mapping.launch.py"
```

---

### 🧭 3. Autonomous Nav2 Navigation
```bash
bash -c "source /opt/ros/humble/setup.bash && source /home/ps/psnh_turtlebot_ws/install/setup.bash && export DISPLAY=:1 && export XAUTHORITY=/home/ps/.Xauthority && exec ros2 launch custom_bot full_navigation.launch.py"
```

---

### 🕹️ 4. Manual SLAM Mapping & Hardware Bringup

#### Full Hardware Bringup
```bash
ros2 launch custom_bot robot_full.launch.py
```

#### SLAM Toolbox + RViz2
```bash
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
ros2 launch custom_bot mapping.launch.py
```

#### Teleoperation (Hold-to-Move Deadman Switch)
```bash
ros2 run custom_bot teleop_key_hold.py
```

---

## 💾 SECTION 3: Saving, Inspecting & Exporting Maps

### 1. Save Map Live While SLAM is Running
Save the current occupancy grid directly to the Desktop:
```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f /home/ps/Desktop/my_office_map
```
*(This generates `/home/ps/Desktop/my_office_map.yaml` and `/home/ps/Desktop/my_office_map.pgm`)*

### 2. Convert Saved PGM Map to PNG (For Viewing or Reports)
```bash
python3 -c "from PIL import Image; Image.open('/home/ps/Desktop/my_office_map.pgm').save('/home/ps/Desktop/my_office_map.png'); print('Converted to PNG!')"
```

### 3. Capture Live RViz2 Desktop Screenshot
Take a high-resolution screenshot of the TigerVNC display without interrupting any nodes:
```bash
DISPLAY=:1 import -window root /home/ps/Desktop/live_rviz_screenshot.png
```

---

## 🔍 SECTION 4: Diagnostics & Live Telemetry Verification

### 1. Check Active Topics & Rates
```bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /scan          # Expected: ~10.0 Hz
ros2 topic hz /odom          # Expected: ~20.0 Hz
ros2 topic hz /tof/range     # Expected: ~20.0 Hz
ros2 topic echo /odom --once # Prints current pose & covariance
```

### 2. Verify Coordinate Transform Tree (TF2)
```bash
# Check transform between map and base_footprint:
ros2 run tf2_ros tf2_echo map base_footprint

# Generate a visual PDF of the full transform hierarchy:
ros2 run tf2_tools view_frames
# (Generates frames.pdf in current directory)
```

### 3. Check Hardware Communication
```bash
# Test Arduino UART directly (baud 115200):
python3 -c "import serial; s=serial.Serial('/dev/ttyAMA0', 115200, timeout=1); print('Arduino Response:', s.readline().decode('utf-8', errors='ignore')); s.close()"

# Scan I2C bus for VL53L0X ToF (should show 0x29):
i2cdetect -y 1

# List CSI Camera Device:
v4l2-ctl --list-devices
```

---

## 🔄 SECTION 5: Git Version Control & Syncing

### Python Package (`nammy_ps_turtlebot3`)
```bash
cd /home/ps/psnh_turtlebot_ws
git status
git pull origin main
# To push changes:
git add .
git commit -m "Your commit message"
git push origin main
```

### C++ Package (`custom_turtlebot3_cpp`)
```bash
cd /home/ps/psnh_turtlebot_cpp_ws
git status
git pull origin main
# To push changes:
git add .
git commit -m "Your commit message"
git push origin main
```

---

## 📋 SECTION 6: Parameter Tuning Quick Reference

| Parameter | Location (C++ / Python) | Default Value | Notes |
| :--- | :--- | :--- | :--- |
| **Encoder Left CPR** | `arduino_bridge.cpp` / `robot_full.launch.py` | `412` | Calibrated for 65mm Rhino wheel on left motor |
| **Encoder Right CPR** | `arduino_bridge.cpp` / `robot_full.launch.py` | `440` | Calibrated for 65mm Rhino wheel on right motor |
| **Track Width (Separation)**| `arduino_bridge.cpp` / `robot_full.launch.py` | `0.150 m` | Calibrated effective turning center width |
| **Crawl Speed PWM** | `arduino_bridge.cpp` / `arduino_bridge.py` | `(92, 98)` | Balanced differential crawl overcome carpet stiction |
| **Laser Max Range** | `ydlidar_ros2_driver` / `robot_full` | `6.0 m` | Filters noisy specular reflections past 6 meters |
| **ToF Max Range** | `vl53l0x_node.cpp` / `vl53l0x_node.py` | `0.45 m` | Bounded to ignore carpet grazing beyond 0.45m |
| **Frontier Obstacle Dist** | `simple_explorer.cpp` / `simple_explorer.py` | `0.38 m` | Wall avoidance trigger distance |
| **Critical Collision Dist** | `simple_explorer.cpp` / `simple_explorer.py` | `0.25 m` | Emergency reverse & spin threshold |

---

*Document compiled on Linux / Raspberry Pi 4B (Humble Hawksbill). Stored at `/home/ps/Desktop/README_COMMANDS.md`.*
