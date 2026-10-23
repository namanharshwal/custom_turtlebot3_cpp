# Custom TurtleBot Burger 2.0 (100% Pure C++ Architecture)

[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![Language](https://img.shields.io/badge/Language-Modern%20C%2B%2B17-00599C?logo=c%2B%2B&logoColor=white)](https://isocpp.org/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%204B%20(2GB)-C51A4A?logo=raspberrypi&logoColor=white)](https://www.raspberrypi.com/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Creator](https://img.shields.io/badge/Creator%20%26%20Maintainer-Naman%20Harshwal-success)](https://github.com/namanharshwal)

* **Creator, Author & Maintainer:** **Naman Harshwal** (`namanharshwal@gmail.com`)
* **Project Owner / Supervisor:** **Pradeep Singh Chauhan** (`ps@robot.com`)
* **Workspace:** `psnh_turtlebot_cpp_ws` (High-Performance 100% Native C++ Stack)
* **License:** Apache License 2.0 (c) 2026 Naman Harshwal

---

## 📌 Executive Overview

This repository provides an ultra-low latency, multi-threaded **100% Pure C++17 native implementation** of the Custom TurtleBot Burger 2.0 autonomous robotics stack.

Every single node, hardware driver, computer vision pipeline, autonomous navigation supervisor, and system orchestrator is compiled directly into native C++ ELF machine binaries. There are **zero Python runtime scripts** in this workspace.

---

## 🏗️ 100% Native C++17 Executable Architecture

| Native C++ Binary | Source File | Key Technical & Algorithmic Capabilities |
| :--- | :--- | :--- |
| `arduino_bridge` | [`arduino_bridge.cpp`](src/custom_bot_cpp/src/arduino_bridge.cpp) | Direct Linux POSIX `termios` UART driver, non-blocking background thread, dual-CPR tick conversions ($412\text{ L}$ vs $440\text{ R}$ ticks/rev), REP 103 yaw coordinate normalization, and Layer 1 PWM anti-stall pulse injection. |
| `simple_explorer` | [`simple_explorer.cpp`](src/custom_bot_cpp/src/simple_explorer.cpp) | Multi-sector laser scan spatial partitioning, deterministic 10 Hz reactive state machine (`FORWARD`, `CURVE`, `ROTATE`, `REVERSE`), Euclidean odometry stall detection, and continuous room coverage. |
| `teleop_key_hold` | [`teleop_key_hold.cpp`](src/custom_bot_cpp/src/teleop_key_hold.cpp) | POSIX `poll()` on `STDIN_FILENO` with 100ms deadman timeout, instantaneous dynamic braking upon key release, and smooth geometric velocity ramping ($\pm10\%$). |
| `object_detector_node` | [`object_detector_node.cpp`](src/custom_bot_cpp/src/object_detector_node.cpp) | Sub-millisecond MIPI RAW10 Bayer unpacking ($< 0.3\text{ ms}$ on ARM Cortex-A72), Gray-World auto-white balance, dynamic contrast normalization, and Darknet YOLOv4-tiny CPU inference across 2 OpenMP threads (< 45 MB RAM). |
| `robot_full_launcher` | [`robot_full_launcher.cpp`](src/custom_bot_cpp/src/robot_full_launcher.cpp) | Native C++ POSIX process supervisor launching Robot State Publisher, YDLidar X2, Arduino Bridge, and Camera Object Detector with signal group forwarding. |
| `slam_launcher` | [`slam_launcher.cpp`](src/custom_bot_cpp/src/slam_launcher.cpp) | Native C++ SLAM orchestrator initializing SLAM Toolbox async node with Ceres scan-matching optimization. |
| `autonomous_mapping_launcher` | [`autonomous_mapping_launcher.cpp`](src/custom_bot_cpp/src/autonomous_mapping_launcher.cpp) | Complete C++ autonomous mapping master: orchestrates hardware bringup, SLAM mapping, RViz2 visualization, and the simple explorer. |
| `full_navigation_launcher` | [`full_navigation_launcher.cpp`](src/custom_bot_cpp/src/full_navigation_launcher.cpp) | Complete C++ autonomous navigation master: orchestrates hardware bringup, Nav2 stack with saved room map, AMCL localization, and RViz2. |
| `waypoint_navigator` | [`waypoint_navigator.cpp`](src/custom_bot_cpp/src/waypoint_navigator.cpp) | Native C++ Nav2 Action Client implementing autonomous waypoint sequence tours (`NavigateThroughPoses`). |
| Firmware (`.cpp`) | [`custom_bot_motor_encoder.cpp`](src/custom_bot_cpp/arduino_firmware/custom_bot_motor_encoder/custom_bot_motor_encoder.cpp) | ATmega328P hardware interrupt quadrature encoder decoding, L298N deadband remapping, and 250ms safety watchdog. |

---

## 🚀 Building & Running

### 1. Build the Workspace
```bash
cd /home/ps/psnh_turtlebot_cpp_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Autonomous Mapping (Pure C++)
```bash
ros2 run custom_bot_cpp autonomous_mapping_launcher
```

### 3. Launch Nav2 Waypoint Navigation with Saved Map (Pure C++)
```bash
ros2 run custom_bot_cpp full_navigation_launcher
```

### 4. Execute Autonomous Waypoint Patrol Tour (Pure C++)
```bash
ros2 run custom_bot_cpp waypoint_navigator
```
