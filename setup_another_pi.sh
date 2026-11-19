#!/usr/bin/env bash
# ==============================================================================
# 🤖 Custom Autonomous TurtleBot Burger 2.0 — Automated System Setup & Restore
# Complete Bare-Metal Deployment Script for Raspberry Pi 4B (Ubuntu 22.04 LTS)
# Creator & Lead Maintainer: Naman Harshwal (namanharshwal@gmail.com)
# Owner / Supervisor:        Pradeep Singh Chauhan (ps@robot.com)
# ==============================================================================

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}====================================================================${NC}"
echo -e "${GREEN} 🚀 Custom TurtleBot Burger 2.0 — Master Pi 4B Setup & Restore${NC}"
echo -e "${BLUE}====================================================================${NC}"

# Check for root / sudo
if [ "$EUID" -eq 0 ]; then
    echo -e "${RED}[ERROR] Please do NOT run this script as root directly. Run as user 'ps' with sudo access.${NC}"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_USER="$USER"
TARGET_HOME="$HOME"

echo -e "${YELLOW}[1/8] Updating package lists and installing core dependencies...${NC}"
sudo apt update
sudo apt install -y curl gnupg2 lsb-release build-essential git cmake software-properties-common

# 2. Add ROS 2 Humble Repository if not present
echo -e "${YELLOW}[2/8] Setting up ROS 2 Humble Hawksbill repositories...${NC}"
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-teleop \
    ros-humble-teleop-twist-keyboard \
    ros-humble-robot-localization \
    ros-humble-tf-transformations \
    ros-humble-tf2-tools \
    ros-humble-v4l2-camera \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-serial \
    python3-opencv \
    libserial-dev \
    tigervnc-standalone-server \
    tigervnc-common \
    xfce4 \
    xfce4-goodies \
    v4l-utils \
    i2c-tools

# Initialize rosdep
if [ ! -d "/etc/ros/rosdep/sources.list.d" ]; then
    sudo rosdep init || true
fi
rosdep update || true

# 3. Restore Hardware Udev Rules & User Permissions
echo -e "${YELLOW}[3/8] Restoring udev hardware rules and device permissions...${NC}"
if [ -d "$SCRIPT_DIR/system_configs/udev_rules" ]; then
    sudo cp "$SCRIPT_DIR/system_configs/udev_rules/"* /etc/udev/rules.d/
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo -e "${GREEN}✓ Udev rules for /dev/ydlidar and /dev/ttyAMA0 restored successfully.${NC}"
fi

# Add user to required groups
sudo usermod -aG dialout,i2c,video,tty "$TARGET_USER"

# 4. Restore YDLidar SDK
echo -e "${YELLOW}[4/8] Installing YDLidar C++ SDK...${NC}"
if [ -d "$SCRIPT_DIR/system_configs/ydlidar_sdk" ]; then
    if [ -f "$SCRIPT_DIR/system_configs/ydlidar_sdk/libydlidar_sdk.a" ]; then
        sudo cp "$SCRIPT_DIR/system_configs/ydlidar_sdk/libydlidar_sdk.a" /usr/local/lib/
    fi
    if [ -f "$SCRIPT_DIR/system_configs/ydlidar_sdk/ydlidar_config.h" ]; then
        sudo cp "$SCRIPT_DIR/system_configs/ydlidar_sdk/ydlidar_config.h" /usr/local/include/
    fi
    if [ -d "$SCRIPT_DIR/system_configs/ydlidar_sdk/core" ]; then
        sudo cp -r "$SCRIPT_DIR/system_configs/ydlidar_sdk/core" /usr/local/include/
    fi
    if [ -d "$SCRIPT_DIR/system_configs/ydlidar_sdk/src" ]; then
        sudo cp -r "$SCRIPT_DIR/system_configs/ydlidar_sdk/src" /usr/local/include/
    fi
    sudo ldconfig
    echo -e "${GREEN}✓ YDLidar SDK library and headers restored to /usr/local.${NC}"
fi

# 5. Restore Raspberry Pi Hardware Boot Overlays
echo -e "${YELLOW}[5/8] Configuring Raspberry Pi hardware overlays (UART, Camera, I2C)...${NC}"
BOOT_CONFIG="/boot/firmware/config.txt"
[ ! -f "$BOOT_CONFIG" ] && BOOT_CONFIG="/boot/config.txt"

if [ -f "$BOOT_CONFIG" ]; then
    grep -q "enable_uart=1" "$BOOT_CONFIG" || sudo bash -c "echo 'enable_uart=1' >> $BOOT_CONFIG"
    grep -q "dtparam=i2c_arm=on" "$BOOT_CONFIG" || sudo bash -c "echo 'dtparam=i2c_arm=on' >> $BOOT_CONFIG"
    grep -q "dtoverlay=ov5647" "$BOOT_CONFIG" || sudo bash -c "echo 'dtoverlay=ov5647' >> $BOOT_CONFIG"
    grep -q "dtoverlay=disable-bt" "$BOOT_CONFIG" || sudo bash -c "echo 'dtoverlay=disable-bt' >> $BOOT_CONFIG"
    echo -e "${GREEN}✓ Hardware configuration updated in $BOOT_CONFIG.${NC}"
fi

# 6. Restore TigerVNC & User Shell Environment
echo -e "${YELLOW}[6/8] Configuring VNC desktop and shell environments...${NC}"
mkdir -p "$TARGET_HOME/.vnc"
if [ -f "$SCRIPT_DIR/system_configs/user_configs/xstartup" ]; then
    cp "$SCRIPT_DIR/system_configs/user_configs/xstartup" "$TARGET_HOME/.vnc/xstartup"
    chmod +x "$TARGET_HOME/.vnc/xstartup"
fi
if [ -f "$SCRIPT_DIR/system_configs/user_configs/config" ]; then
    cp "$SCRIPT_DIR/system_configs/user_configs/config" "$TARGET_HOME/.vnc/config"
fi

# Environment configuration in .bashrc
grep -q "/opt/ros/humble/setup.bash" "$TARGET_HOME/.bashrc" || cat << 'BASHRC_EOF' >> "$TARGET_HOME/.bashrc"

# Custom TurtleBot Burger 2.0 Environment
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export DISPLAY=:1
export XAUTHORITY=/home/ps/.Xauthority
BASHRC_EOF

# Restore Desktop Manual
mkdir -p "$TARGET_HOME/Desktop"
if [ -f "$SCRIPT_DIR/system_configs/desktop_manuals/README_COMMANDS.md" ]; then
    cp "$SCRIPT_DIR/system_configs/desktop_manuals/README_COMMANDS.md" "$TARGET_HOME/Desktop/"
fi

# 7. Extract Workspaces
echo -e "${YELLOW}[7/8] Extracting C++ and Python robot workspaces...${NC}"
if [ -f "$SCRIPT_DIR/workspace_cpp.tar.gz" ]; then
    echo "Extracting C++ workspace to $TARGET_HOME/psnh_turtlebot_cpp_ws..."
    tar -xzf "$SCRIPT_DIR/workspace_cpp.tar.gz" -C "$TARGET_HOME"
fi
if [ -f "$SCRIPT_DIR/workspace_py.tar.gz" ]; then
    echo "Extracting Python workspace to $TARGET_HOME/psnh_turtlebot_ws..."
    tar -xzf "$SCRIPT_DIR/workspace_py.tar.gz" -C "$TARGET_HOME"
fi

# 8. Build Workspaces
echo -e "${YELLOW}[8/8] Building robot workspaces with colcon...${NC}"
source /opt/ros/humble/setup.bash

if [ -d "$TARGET_HOME/psnh_turtlebot_cpp_ws" ]; then
    echo "Building C++ workspace..."
    cd "$TARGET_HOME/psnh_turtlebot_cpp_ws"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

if [ -d "$TARGET_HOME/psnh_turtlebot_ws" ]; then
    echo "Building Python workspace..."
    cd "$TARGET_HOME/psnh_turtlebot_ws"
    colcon build --symlink-install
fi

echo -e "${BLUE}====================================================================${NC}"
echo -e "${GREEN} 🎉 SETUP COMPLETE! Custom TurtleBot Burger 2.0 is fully restored!${NC}"
echo -e "${YELLOW} Next steps:${NC}"
echo -e "  1. Reboot your Raspberry Pi: ${GREEN}sudo reboot${NC}"
echo -e "  2. Start TigerVNC server:   ${GREEN}vncserver :1${NC}"
echo -e "  3. Launch mapping stack:     See ${BLUE}~/Desktop/README_COMMANDS.md${NC}"
echo -e "${BLUE}====================================================================${NC}"
