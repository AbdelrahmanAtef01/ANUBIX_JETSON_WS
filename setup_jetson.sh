#!/bin/bash
# =============================================================================
# ANUBIX ROS 2 Workspace — Jetson Orin Nano Setup
# =============================================================================
# Run once after flashing JetPack 5.x / 6.x
#
# Prerequisites:
#   - JetPack 5.1.2+ (Ubuntu 20.04) or JetPack 6.x (Ubuntu 22.04)
#   - Internet connection during setup
#
# Usage:
#   chmod +x setup_jetson.sh
#   ./setup_jetson.sh
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR"
LOG_FILE="$WS_DIR/setup_jetson.log"

log() { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG_FILE"; }
die() { log "ERROR: $*"; exit 1; }

log "============================================================"
log "  ANUBIX ROS 2 Workspace Setup — Jetson Orin Nano"
log "  Log: $LOG_FILE"
log "============================================================"

# ── 1. ROS 2 Humble ──────────────────────────────────────────────────────────
log ""
log "[1/6] Checking ROS 2 Humble..."

if [ ! -f /opt/ros/humble/setup.bash ]; then
    log "  Installing ROS 2 Humble..."

    sudo apt update && sudo apt install -y locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt install -y software-properties-common curl
    sudo add-apt-repository universe
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt install -y \
        ros-humble-ros-base \
        ros-humble-rmw-cyclonedds-cpp \
        || die "ROS 2 Humble install failed"

    log "  ROS 2 Humble installed."
else
    log "  ROS 2 Humble already installed."
fi

# ── 2. Build tools and ROS dependencies ──────────────────────────────────────
log ""
log "[2/6] Installing build tools and ROS packages..."

sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    || die "Apt install failed"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init || true
fi
rosdep update

# ── 3. Python dependencies ────────────────────────────────────────────────────
log ""
log "[3/6] Installing Python dependencies..."
pip3 install "omnilink>=0.6.0" numpy pyserial supabase \
    || die "pip install failed"

# ── 4. Build workspace ────────────────────────────────────────────────────────
log ""
log "[4/6] Building ANUBIX Jetson workspace..."
source /opt/ros/humble/setup.bash

cd "$WS_DIR"
rosdep install --from-paths src --ignore-src -r -y \
    || log "  WARNING: rosdep had errors (may be OK for placeholder packages)"

colcon build --symlink-install \
    --packages-select \
        anubix_master \
        anubix_arm \
        anubix_spectrometer \
        anubix_jetson_bridge \
        anubix_supabase \
        anubix_bringup \
    2>&1 | tee -a "$LOG_FILE" \
    || die "colcon build failed"

log "  Build complete."

# ── 5. DDS and ROS environment ────────────────────────────────────────────────
log ""
log "[5/6] Configuring DDS and environment..."

DDS_CONFIG_DST="$HOME/.ros/jetson_cyclone.xml"
mkdir -p "$HOME/.ros"
cp "$WS_DIR/dds_config/jetson_cyclone.xml" "$DDS_CONFIG_DST"
log "  DDS config installed to $DDS_CONFIG_DST"
log "  NOTE: Edit $DDS_CONFIG_DST and set the correct network interface name."
log "        Run 'ip link show' to find your USB-C ethernet interface."

# ── 6. Shell environment ──────────────────────────────────────────────────────
log ""
log "[6/6] Configuring ~/.bashrc..."

BASHRC="$HOME/.bashrc"
if ! grep -q "anubix_ws" "$BASHRC" 2>/dev/null; then
    cat >> "$BASHRC" << 'EOF'

# ── ANUBIX ROS 2 — Jetson ─────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
source ~/anubix_ws/install/setup.bash

# CycloneDDS: unicast-only config for direct Jetson↔RPi ethernet link
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.ros/jetson_cyclone.xml

# All nodes on both machines must share the same domain ID
export ROS_DOMAIN_ID=42

# Suppress multicast to avoid broadcast storms on the direct link
export ROS_LOCALHOST_ONLY=0
# ─────────────────────────────────────────────────────────────────────────────
EOF
    log "  Added ANUBIX environment to ~/.bashrc"
else
    log "  ~/.bashrc already contains ANUBIX entries — skipping."
fi

log ""
log "============================================================"
log "  Setup complete!"
log ""
log "  NEXT STEPS:"
log "  1. Find your USB-C ethernet interface:"
log "       ip link show"
log "  2. Edit the interface name in:"
log "       ~/.ros/jetson_cyclone.xml"
log "     (look for: <NetworkInterface name=\"eth0\">)"
log ""
log "  3. Configure the ethernet link:"
log "       sudo ./configure_jetson_network.sh"
log ""
log "  4. Source and launch:"
log "       source ~/.bashrc"
log "       export OMNI_KEY=olink_YOUR_KEY_HERE"
log "       ros2 launch anubix_bringup jetson.launch.py"
log ""
log "  On Raspberry Pi:"
log "       ros2 launch anubix_bringup_rpi rpi_full.launch.py"
log "============================================================"
