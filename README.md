# Custom TurtleBot Burger 2.0 (High-Performance C++ Architecture)

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-Modern%20C%2B%2B17-00599C?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%20(2GB)-C51A4A?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Creator](https://img.shields.io/badge/Creator%20%26%20Maintainer-Naman%20Harshwal-success)](https://github.com/namanharshwal)

* **Creator, Author & Maintainer:** **Naman Harshwal** (`namanharshwal@gmail.com`)
* **Project Owner / Supervisor:** **Pradeep Singh Chauhan** (`ps@robot.com`)
* **Workspace:** `psnh_turtlebot_cpp_ws` (Local High-Performance C++ Stack)
* **License:** Apache License 2.0 (c) 2026 Naman Harshwal

---

## 📌 Executive Overview

This directory (`psnh_turtlebot_cpp_ws`) provides an ultra-low latency, multi-threaded **C++17 native rewrite** of the Custom TurtleBot Burger 2.0 robotics stack.

By migrating from Python (`rclpy`) to native C++ (`rclcpp`) with POSIX low-level serial I/O, multi-threaded executor workers, and optimized OpenCV C++ DNN inference, CPU utilization on the **Raspberry Pi 4B (2GB)** is reduced by over **65%**, while odometry jitter and message latency are minimized to sub-millisecond levels.

---

## 🏗️ Architecture & Component Mapping

| Python Node (`psnh_turtlebot_ws`) | C++ Native Node (`psnh_turtlebot_cpp_ws`) | Key C++ Optimizations |
| :--- | :--- | :--- |
| `arduino_bridge.py` | `arduino_bridge` (`arduino_bridge.cpp`) | Direct Linux POSIX `termios` UART driver, non-blocking background thread, decoupled CPR tick conversions ($412$ left, $440$ right), REP 103 yaw sign inversion, Layer 1 anti-stall pulse injection. |
| `simple_explorer.py` | `simple_explorer` (`simple_explorer.cpp`) | Zero-copy `SensorDataQoS` laser scan sector analysis, deterministic 10 Hz state machine (`FORWARD`, `CURVE`, `ROTATE`, `REVERSE`), odometry Euclidean stall recovery, sinusoidal exploration wander. |
| `teleop_key_hold.py` | `teleop_key_hold` (`teleop_key_hold.cpp`) | POSIX `poll()` on `STDIN_FILENO` with 100ms deadman timeout, instantaneous dynamic braking upon key release, geometric speed scaling ($\pm10\%$). |
| `object_detector_node.py` | `object_detector_node` (`object_detector_node.cpp`) | Decoupled V4L2 capture and OpenCV DNN YOLOv4-tiny worker threads running at 2.0 Hz with NMS IoU filtering, zero frame lag, minimal RAM footprint (< 40 MB). |

---

## 🚀 Building & Running

### 1. Build the Workspace
```bash
cd /home/ps/psnh_turtlebot_cpp_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Full Hardware Stack & SLAM Mapping
```bash
# Terminal 1: Hardware stack (C++ Arduino bridge + LiDAR + State Publisher + Camera)
ros2 launch custom_bot_cpp robot_full.launch.py

# Terminal 2: SLAM Toolbox
ros2 launch custom_bot_cpp slam.launch.py

# Terminal 3: Autonomous Exploration or Teleoperation
ros2 run custom_bot_cpp simple_explorer
# OR
ros2 run custom_bot_cpp teleop_key_hold
```

### 3. Launch Nav2 Autonomous Navigation
```bash
ros2 launch custom_bot_cpp full_navigation.launch.py
```
