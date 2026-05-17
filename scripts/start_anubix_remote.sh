#!/usr/bin/env bash
#
# start_anubix_remote.sh — start the ANUBIX Jetson stack so an operator on a
# different network can drive it through the hosted OmniLink web UI with zero
# laptop-side setup.
#
# This is the "100% will work" path from OmniLink Remote Agent Access v1.0.0
# §5.1 (Option E — HTTPS Tunnel). The master node starts a Cloudflare quick
# tunnel, re-registers the live URL into the ANUBIX agent profile, and then
# launches the rest of the stack via the bringup launch file.
#
# Prerequisites on the Jetson:
#   - ROS 2 Humble sourced
#   - anubix_ws built and sourced (source install/setup.bash)
#   - cloudflared installed (run scripts/install_cloudflared.sh once)
#   - OMNI_KEY exported (or set in master_params.yaml)
#
# Usage:
#   export OMNI_KEY=olink_...
#   ./start_anubix_remote.sh

set -euo pipefail

if [[ -z "${OMNI_KEY:-}" ]]; then
    echo "[ERR] OMNI_KEY env var not set. Export it first:" >&2
    echo "        export OMNI_KEY=olink_..." >&2
    exit 1
fi

if ! command -v cloudflared >/dev/null 2>&1; then
    echo "[ERR] cloudflared not found. Run: sudo ./install_cloudflared.sh" >&2
    exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[ERR] ros2 not found. Source ROS 2 first:" >&2
    echo "        source /opt/ros/humble/setup.bash" >&2
    echo "        source <anubix_ws>/install/setup.bash" >&2
    exit 1
fi

echo "[..] Starting ANUBIX Jetson stack (with cloudflared quick tunnel)"
echo "     OMNI_KEY:   ${OMNI_KEY:0:15}..."
echo "     ROS_DOMAIN: ${ROS_DOMAIN_ID:-(unset, defaults to 0)}"
echo

# The master node owns the tunnel. The bringup launch file starts the master
# node along with arm / vision / spectrometer / supabase / jetson_bridge.
exec ros2 launch anubix_bringup jetson.launch.py
