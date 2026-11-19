# 🤖 Custom TurtleBot Burger 2.0 — New Raspberry Pi 4B Complete Setup & Restore Guide
### One-Command Full Bare-Metal Deployment & Backup Restoration Manual

---

## 👨‍💻 Project Authorship & Metadata

* **Author, Creator & Lead Maintainer:** **Naman Harshwal** ([@namanharshwal](https://github.com/namanharshwal) | `namanharshwal@gmail.com`)
* **Project Owner / Supervisor:** **Pradeep Singh Chauhan** (`ps@robot.com`)
* **Target Hardware:** Raspberry Pi 4B (2GB / 4GB / 8GB)
* **Target Operating System:** Ubuntu 22.04 LTS (Jammy Jellyfish) 64-bit
* **Target ROS Distribution:** ROS 2 Humble Hawksbill
* **Workspaces Restored:**
  * C++ High-Performance Workspace: `~/psnh_turtlebot_cpp_ws`
  * Python Modular Workspace: `~/psnh_turtlebot_ws`

---

## 📦 Contents of the Backup Bundle

| File / Folder | Size | Description |
| :--- | :--- | :--- |
| **`setup_another_pi.sh`** | 6.5 KB | **Master One-Command Automated Setup Script** |
| **`workspace_cpp.tar.gz`** | 76 MB | Complete native C++17 workspace (source, models, description, maps, configs) |
| **`workspace_py.tar.gz`** | 65 MB | Complete Python workspace (launch files, scripts, ToF drivers, configs, maps) |
| **`system_configs.tar.gz`** | 598 KB | Udev hardware rules, boot overlays, VNC xstartup, YDLidar SDK, manuals |
| **`Custom_TurtleBot_Burger_2.0_Complete_Backup.tar.gz.part_aa`** | 50 MB | Master archive chunk Part 1 (fits free cloud storage limits) |
| **`Custom_TurtleBot_Burger_2.0_Complete_Backup.tar.gz.part_ab`** | 50 MB | Master archive chunk Part 2 |
| **`Custom_TurtleBot_Burger_2.0_Complete_Backup.tar.gz.part_ac`** | 41 MB | Master archive chunk Part 3 |

---

## ⚡ Step-by-Step Restoration on a New Raspberry Pi 4B

### Step 1: Flash Ubuntu 22.04 LTS (64-bit)
1. Use **Raspberry Pi Imager** to flash `Ubuntu Server 22.04 LTS (64-bit)` onto your MicroSD card or SSD.
2. Set default username to `ps` and password as desired.

---

### Step 2: Download & Extract the Backup Archive

#### If you have the chunked parts (`.part_aa`, `.part_ab`, `.part_ac`):
Reassemble the master archive with a single command:
```bash
cat Custom_TurtleBot_Burger_2.0_Complete_Backup.tar.gz.part_* > Custom_TurtleBot_Burger_2.0_Complete_Backup.tar.gz
tar -xzf Custom_TurtleBot_Burger_2.0_Complete_Backup.tar.gz
cd Custom_TurtleBot_Burger_2.0_Pi4B_Backup
```

#### Or if you have the extracted folder directly:
```bash
cd ~/Custom_TurtleBot_Burger_2.0_Pi4B_Backup
```

---

### Step 3: Run the Master One-Command Installer
Make sure the script is executable and execute it:
```bash
chmod +x setup_another_pi.sh
./setup_another_pi.sh
```

#### What `setup_another_pi.sh` automatically performs:
1. Configures ROS 2 Humble repositories, GPG keys, and apt mirrors.
2. Installs all required ROS 2 packages (`slam-toolbox`, `navigation2`, `nav2-bringup`, `turtlebot3`, `v4l2-camera`, `image-transport`, etc.).
3. Installs TigerVNC, XFCE desktop environment, and build tools (`colcon`, `libserial-dev`, `python3-pip`, `cmake`).
4. Restores custom udev hardware rules (`/dev/ydlidar`, `/dev/ttyAMA0`) and user group permissions (`dialout`, `i2c`, `video`, `tty`).
5. Installs the native YDLidar C++ SDK into `/usr/local/lib` and `/usr/local/include`.
6. Configures `/boot/firmware/config.txt` for UART, I2C, SPI, and OV5647 CSI camera auto-detection.
7. Restores `.bashrc` environment variables and TigerVNC resolution (`1920x1080`).
8. Extracts both workspaces to `~/psnh_turtlebot_cpp_ws` and `~/psnh_turtlebot_ws`.
9. Compiles and verifies both workspaces using `colcon build --symlink-install`.
10. Restores the command reference manual onto the Desktop (`~/Desktop/README_COMMANDS.md`).

---

### Step 4: Reboot the System
To apply hardware device tree overlays (UART, Camera, and I2C), reboot the Raspberry Pi:
```bash
sudo reboot
```

---

## 🔍 Verification Checklist (Post-Reboot)

After rebooting, verify that all hardware devices are recognized:

```bash
# 1. Verify LiDAR symlink:
ls -l /dev/ydlidar
# Expected: /dev/ydlidar -> ttyUSB0 (or ttyACM0)

# 2. Verify Arduino serial port:
ls -l /dev/ttyAMA0
# Expected: crw-rw---- 1 root dialout ... /dev/ttyAMA0

# 3. Verify VL53L0X ToF on I2C bus:
i2cdetect -y 1
# Expected: Address 29 should be detected

# 4. Verify OV5647 CSI Camera:
v4l2-ctl --list-devices
# Expected: unicam or ov5647 device listed under /dev/video0
```

---

## 🚀 Running Autonomous Mapping on the New Robot

### Option 1: Native C++ Stack (Recommended)
```bash
source /opt/ros/humble/setup.bash
source ~/psnh_turtlebot_cpp_ws/install/setup.bash
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
ros2 run custom_bot_cpp autonomous_mapping_launcher
```

### Option 2: Python Stack
```bash
source /opt/ros/humble/setup.bash
source ~/psnh_turtlebot_ws/install/setup.bash
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
ros2 launch custom_bot autonomous_mapping.launch.py
```

*For all other commands (teleoperation, Nav2 navigation, map saving), refer to the desktop manual at `~/Desktop/README_COMMANDS.md`.*
