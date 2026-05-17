#!/usr/bin/env bash
#
# install_cloudflared.sh — one-time install of the Cloudflare Tunnel daemon
# on the Jetson, used by the ANUBIX master node to publish its tool-callback
# server over a public HTTPS URL.
#
# Grounded in OmniLink Remote Agent Access v1.0.0 §5.1 (Option E — HTTPS
# Tunnel). Quick tunnels require no Cloudflare account and produce a real
# Let's-Encrypt-backed HTTPS URL on every run, which is exactly what the
# OmniLink hosted UI needs to bypass the browser's mixed-content rule (§3).
#
# Usage (on the Jetson):
#   chmod +x install_cloudflared.sh
#   sudo ./install_cloudflared.sh
#
# Idempotent: skips download if cloudflared is already on PATH.

set -euo pipefail

if command -v cloudflared >/dev/null 2>&1; then
    echo "[OK] cloudflared already installed: $(cloudflared --version)"
    exit 0
fi

ARCH="$(uname -m)"
case "$ARCH" in
    aarch64|arm64)
        BIN="cloudflared-linux-arm64"
        ;;
    x86_64|amd64)
        BIN="cloudflared-linux-amd64"
        ;;
    armv7l)
        BIN="cloudflared-linux-arm"
        ;;
    *)
        echo "[ERR] Unsupported architecture: $ARCH" >&2
        exit 1
        ;;
esac

URL="https://github.com/cloudflare/cloudflared/releases/latest/download/${BIN}"
DEST="/usr/local/bin/cloudflared"

echo "[..] Downloading cloudflared for $ARCH from $URL"
sudo curl -fL "$URL" -o "$DEST"
sudo chmod +x "$DEST"

echo "[OK] Installed: $($DEST --version)"
echo
echo "Next: run the master node with --tunnel cloudflared (the default)."
echo "  export OMNI_KEY=olink_..."
echo "  ros2 run anubix_master master_node --tunnel cloudflared"
echo
echo "The first connection to a quick-tunnel URL can take ~10s to propagate."
