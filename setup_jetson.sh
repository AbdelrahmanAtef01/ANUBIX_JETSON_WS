#!/bin/bash
# =============================================================================
# ANUBIX ROS 2 Workspace - Jetson Orin Nano Setup
# =============================================================================
# Run this script on the Jetson Orin Nano after flashing JetPack 5.x/6.x
#
# Prerequisites:
#   - JetPack 5.1.2+ (Ubuntu 20.04) or JetPack 6.x (Ubuntu 22.04)
#   - Internet connection
#
# Usage:
#   chmod +x setup_jetson.sh
#   ./setup_jetson.sh
# =============================================================================

set -e

echo "============================================="
echo "  ANUBIX ROS 2 Workspace Setup"
echo "  Target: Jetson Orin Nano"
echo "============================================="

# --- ROS 2 Humble Installation ---
echo ""
echo "[1/5] Checking ROS 2 Humble installation..."

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "  ROS 2 Humble not found. Installing..."

    # Set locale
    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    # Add ROS 2 apt repo
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install -y curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y ros-humble-ros-base ros-humble-rmw-cyclonedds-cpp

    echo "  ROS 2 Humble installed."
else
    echo "  ROS 2 Humble found."
fi

# --- Dev tools ---
echo ""
echo "[2/5] Installing build tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav2-msgs

# Initialize rosdep if needed
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

# --- Python dependencies ---
echo ""
echo "[3/5] Installing Python dependencies..."
pip3 install omnilink>=0.6.0

# --- Build workspace ---
echo ""
echo "[4/5] Building ANUBIX workspace..."
source /opt/ros/humble/setup.bash

cd "$(dirname "$0")"
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# --- Shell setup ---
echo ""
echo "[5/5] Configuring shell environment..."

SETUP_LINE='source ~/anubix_ws/install/setup.bash'
BASHRC="$HOME/.bashrc"

if ! grep -q "anubix_ws" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ANUBIX ROS 2 Workspace" >> "$BASHRC"
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
    echo "$SETUP_LINE" >> "$BASHRC"
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> "$BASHRC"
    echo "  Added workspace source to ~/.bashrc"
fi

echo ""
echo "============================================="
echo "  Setup Complete!"
echo ""
echo "  To run the full system:"
echo "    source install/setup.bash"
echo "    export OMNI_KEY=olink_YOUR_KEY_HERE"
echo "    ros2 launch anubix_bringup anubix_full.launch.py"
echo ""
echo "  To run master node only:"
echo "    ros2 run anubix_master master_node"
echo ""
echo "  To run individual stacks:"
echo "    ros2 run anubix_navigation nav_node"
echo "    ros2 run anubix_perception perception_node"
echo "    ros2 run anubix_arm arm_node"
echo "    ros2 run anubix_spectrometer spectrometer_node"
echo "============================================="
