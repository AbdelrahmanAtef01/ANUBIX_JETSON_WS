#!/bin/bash
# =============================================================================
# ANUBIX — Jetson Orin Nano  ·  One-Command Setup
# =============================================================================
#
# Run ONE command on a fresh Jetson (JetPack 5.x / 6.x):
#
#   bash <(curl -fsSL https://raw.githubusercontent.com/AbdelrahmanAtef01/ANUBIX_JETSON_WS/main/install_jetson.sh)
#
# Or if you already cloned:
#
#   chmod +x install_jetson.sh && ./install_jetson.sh
#
# What this script does (fully automatic, no extra input needed):
#   1.  Clones ANUBIX_JETSON_WS → ~/anubix_ws   (skip if already present)
#   2.  Installs ROS 2 Humble                   (skip if already present)
#   3.  Installs build tools (colcon, rosdep)
#   4.  Installs Python dependencies            (omnilink, supabase, ultralytics …)
#   5.  Builds the workspace with colcon
#   6.  Auto-detects the USB-C ethernet adapter and assigns 192.168.10.1/24
#   7.  Writes a persistent netplan config      (survives reboots)
#   8.  Copies & patches the CycloneDDS XML with the real interface name
#   9.  Writes all env vars to ~/.bashrc        (ROS, DDS, OMNI_KEY hardcoded)
#  10.  Prints the single launch command
#
# After this script finishes just run:
#   source ~/.bashrc
#   ros2 launch anubix_bringup jetson.launch.py
# =============================================================================

set -o pipefail

# ── Hardcoded credentials (from project files) ───────────────────────────────
OMNI_KEY="olink_4ekYIgHACfZaGlq6WJOgu59U"
SUPABASE_URL="https://bdkutmmrcjckaazzzspe.supabase.co"
SUPABASE_KEY="sb_publishable_VY6-Jjc6f20Wcbb3Rm8gwg_ZK6CYuh3"

# ── Constants ─────────────────────────────────────────────────────────────────
REPO_URL="https://github.com/AbdelrahmanAtef01/ANUBIX_JETSON_WS.git"
WS_DIR="$HOME/anubix_ws"
LOG_FILE="$WS_DIR/install_jetson.log"
JETSON_IP="192.168.10.1"
RPI_IP="192.168.10.2"

# ── Logging ───────────────────────────────────────────────────────────────────
mkdir -p "$WS_DIR" 2>/dev/null || true
log()  { echo "[$(date '+%H:%M:%S')] $*" | tee -a "$LOG_FILE"; }
warn() { log "WARNING: $*"; }
die()  { log "ERROR: $*"; exit 1; }

log "============================================================"
log "  ANUBIX Jetson Orin Nano — Full Setup"
log "  Log: $LOG_FILE"
log "============================================================"

# ── 0. Prerequisites ──────────────────────────────────────────────────────────
log ""
log "[0/9] Installing prerequisites (git, curl) ..."
sudo apt-get update -qq
sudo apt-get install -y git curl > /dev/null

# ── 1. Clone workspace ────────────────────────────────────────────────────────
log ""
log "[1/9] Setting up workspace at $WS_DIR ..."

if [ -d "$WS_DIR/.git" ]; then
    log "  Workspace already cloned — pulling latest..."
    git -C "$WS_DIR" pull --ff-only || warn "git pull failed — continuing with existing code"
else
    log "  Cloning $REPO_URL → $WS_DIR ..."
    git clone "$REPO_URL" "$WS_DIR" || die "git clone failed"
fi

# Redirect log file inside the now-existing workspace
LOG_FILE="$WS_DIR/install_jetson.log"
cd "$WS_DIR"

# ── 2. ROS 2 Humble ───────────────────────────────────────────────────────────
log ""
log "[2/9] Checking ROS 2 Humble ..."

if [ ! -f /opt/ros/humble/setup.bash ]; then
    log "  Installing ROS 2 Humble (this takes ~5 minutes) ..."

    sudo apt-get install -y locales > /dev/null
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    sudo apt-get install -y software-properties-common > /dev/null
    sudo add-apt-repository -y universe > /dev/null
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y \
        ros-humble-ros-base \
        ros-humble-rmw-cyclonedds-cpp \
        || die "ROS 2 Humble install failed"
    log "  ROS 2 Humble installed."
else
    log "  ROS 2 Humble already installed."
fi

# ── 3. Build tools ────────────────────────────────────────────────────────────
log ""
log "[3/9] Installing build tools ..."

sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    python3-opencv \
    || die "apt install failed"

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init 2>/dev/null || true
fi
rosdep update 2>&1 | tail -5

# ── 4. Python dependencies ────────────────────────────────────────────────────
log ""
log "[4/9] Installing Python dependencies ..."

pip3 install --quiet \
    "omnilink>=0.6.0" \
    numpy \
    pyserial \
    supabase \
    ultralytics \
    || die "pip install failed"

log "  NOTE: pyrealsense2 (RealSense SDK) is NOT on PyPI for arm64."
log "        Camera 1 (RealSense depth) will be inactive unless you install it:"
log "        sudo apt install librealsense2-dev python3-pyrealsense2"

# ── 5. Build workspace ────────────────────────────────────────────────────────
log ""
log "[5/9] Building ANUBIX Jetson workspace (this takes ~2 minutes) ..."

. /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y 2>&1 | tail -5 \
    || warn "rosdep had errors (may be OK for placeholder packages)"

colcon build --symlink-install \
    --packages-select \
        anubix_master \
        anubix_arm \
        anubix_spectrometer \
        anubix_vision \
        anubix_jetson_bridge \
        anubix_supabase \
        anubix_bringup \
    2>&1 | tee -a "$LOG_FILE" \
    || die "colcon build failed — see $LOG_FILE for details"

log "  Build complete."

# ── 6. Network configuration ─────────────────────────────────────────────────
log ""
log "[6/9] Configuring direct ethernet link (Jetson → 192.168.10.1) ..."

# Auto-detect USB-C / USB ethernet adapter
IFACE=""
for candidate in $(ip -o link show | awk -F': ' '{print $2}' | grep -v lo); do
    if [[ "$candidate" =~ ^(usb|enx|enp) ]]; then
        IFACE="$candidate"
        break
    fi
done
if [ -z "$IFACE" ]; then
    # Fallback: first non-loopback non-wifi interface
    IFACE=$(ip -o link show | awk -F': ' '{print $2}' \
        | grep -v lo | grep -v '^wl' | head -1)
fi

if [ -z "$IFACE" ]; then
    warn "Cannot auto-detect ethernet interface."
    warn "After setup, run: sudo ip addr add 192.168.10.1/24 dev <YOUR_IFACE>"
    warn "And edit: ~/.ros/jetson_cyclone.xml  (replace eth0 with your interface name)"
    IFACE="eth0"
else
    log "  Detected interface: $IFACE"
    sudo ip addr flush dev "$IFACE" 2>/dev/null || true
    sudo ip addr add "${JETSON_IP}/24" dev "$IFACE" 2>/dev/null || true
    sudo ip link set "$IFACE" up

    # Persist with netplan
    if command -v netplan &>/dev/null; then
        sudo tee /etc/netplan/99-anubix-link.yaml > /dev/null << NETPLAN
network:
  version: 2
  ethernets:
    ${IFACE}:
      dhcp4: false
      addresses:
        - ${JETSON_IP}/24
      optional: true
NETPLAN
        sudo netplan apply 2>/dev/null || warn "netplan apply failed"
        log "  Static IP ${JETSON_IP}/24 set on $IFACE (persists across reboots)"
    fi

    # Quick ping test (best-effort)
    if ping -c 2 -W 2 -I "$IFACE" "$RPI_IP" > /dev/null 2>&1; then
        log "  Raspberry Pi ($RPI_IP) is reachable — link OK"
    else
        log "  Raspberry Pi not reachable yet (run install_rpi.sh on the RPi first)"
    fi
fi

# ── 7. DDS configuration ──────────────────────────────────────────────────────
log ""
log "[7/9] Installing CycloneDDS config ..."

mkdir -p "$HOME/.ros"
cp "$WS_DIR/dds_config/jetson_cyclone.xml" "$HOME/.ros/jetson_cyclone.xml"

# Patch the interface name in the XML to match what we detected
if [ "$IFACE" != "eth0" ]; then
    sed -i "s/name=\"eth0\"/name=\"${IFACE}\"/" "$HOME/.ros/jetson_cyclone.xml"
    log "  DDS interface patched: eth0 → $IFACE"
else
    log "  DDS interface left as eth0 — verify with: ip link show"
fi
log "  DDS config: $HOME/.ros/jetson_cyclone.xml"

# ── 8. Shell environment ──────────────────────────────────────────────────────
log ""
log "[8/9] Writing environment to ~/.bashrc ..."

BASHRC="$HOME/.bashrc"
if ! grep -q "anubix_ws" "$BASHRC" 2>/dev/null; then
    # Write block with heredoc — keys are expanded here intentionally
    cat >> "$BASHRC" << BASHRC_BLOCK

# ── ANUBIX ROS 2 — Jetson Orin Nano ──────────────────────────────────────────
source /opt/ros/humble/setup.bash
source \$HOME/anubix_ws/install/setup.bash

# CycloneDDS unicast config for direct Jetson↔RPi ethernet link
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://\$HOME/.ros/jetson_cyclone.xml

# Both machines must share the same domain ID
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# ANUBIX service credentials
export OMNI_KEY=${OMNI_KEY}
export SUPABASE_URL=${SUPABASE_URL}
export SUPABASE_KEY=${SUPABASE_KEY}
# ─────────────────────────────────────────────────────────────────────────────
BASHRC_BLOCK
    log "  Environment block written to ~/.bashrc"
else
    log "  ~/.bashrc already contains ANUBIX entries — skipping (re-run if keys changed)"
fi

# ── 9. Done ───────────────────────────────────────────────────────────────────
log ""
log "============================================================"
log "  ANUBIX JETSON SETUP COMPLETE"
log "============================================================"
log ""
log "  To launch (run these 2 commands):"
log ""
log "    source ~/.bashrc"
log "    ros2 launch anubix_bringup jetson.launch.py"
log ""
log "  Before first real run:"
log "    - Copy best.engine to ~/anubix_ws/ (TensorRT model)"
log "      yolo export model=best.pt format=engine half=true"
log "    - Verify ethernet link:  ping 192.168.10.2"
log "    - Check DDS interface:   cat ~/.ros/jetson_cyclone.xml"
log ""
log "  Log saved to: $LOG_FILE"
log "============================================================"
