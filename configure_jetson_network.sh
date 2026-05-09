#!/bin/bash
# =============================================================================
# Jetson Orin Nano — Direct Ethernet to Raspberry Pi
# =============================================================================
# Configures the USB-C ethernet adapter interface with a static IP for
# the point-to-point link to the Raspberry Pi.
#
# Jetson  → 192.168.10.1/24
# RPi     → 192.168.10.2/24
#
# Usage:
#   sudo ./configure_jetson_network.sh [INTERFACE]
#
# INTERFACE defaults to auto-detect. Override with e.g.:
#   sudo ./configure_jetson_network.sh usb0
#   sudo ./configure_jetson_network.sh eth0
# =============================================================================

set -euo pipefail

JETSON_IP="192.168.10.1"
RPI_IP="192.168.10.2"
PREFIX="24"

log() { echo "[$(date '+%H:%M:%S')] $*"; }
die() { log "ERROR: $*"; exit 1; }

# ── Detect interface ──────────────────────────────────────────────────────────
if [ -n "${1:-}" ]; then
    IFACE="$1"
    log "Using specified interface: $IFACE"
else
    # Auto-detect: find a non-loopback, non-wireless interface that is NOT
    # the primary Ethernet (skip eth0 on systems where it exists as built-in).
    # USB-C ethernet adapters often appear as usb0, enx<mac>, or an enumerated
    # name. We pick the first non-lo, non-wl interface that is currently UP
    # or has carrier — prefer names starting with usb, enx, enp.
    IFACE=""
    for iface in $(ip -o link show | awk -F': ' '{print $2}' | grep -v lo); do
        name="$iface"
        if [[ "$name" =~ ^(usb|enx|enp) ]]; then
            IFACE="$name"
            break
        fi
    done

    if [ -z "$IFACE" ]; then
        # Fallback: first non-loopback, non-wlan interface
        IFACE=$(ip -o link show | awk -F': ' '{print $2}' \
            | grep -v lo | grep -v '^wl' | head -1)
    fi

    [ -n "$IFACE" ] || die "Could not auto-detect ethernet interface. " \
        "Pass it explicitly: sudo $0 <interface>"

    log "Auto-detected interface: $IFACE"
    log "If wrong, re-run: sudo $0 <correct_interface>"
fi

# ── Validate interface exists ─────────────────────────────────────────────────
ip link show "$IFACE" > /dev/null 2>&1 \
    || die "Interface '$IFACE' not found. Run 'ip link show' to list interfaces."

log "Configuring $IFACE as Jetson side of direct ethernet link..."

# ── Remove any existing address and bring interface up ────────────────────────
ip addr flush dev "$IFACE" 2>/dev/null || true
ip addr add "${JETSON_IP}/${PREFIX}" dev "$IFACE"
ip link set "$IFACE" up

log "Assigned ${JETSON_IP}/${PREFIX} to $IFACE"

# ── Wait for link ─────────────────────────────────────────────────────────────
log "Waiting for link carrier..."
for i in $(seq 1 10); do
    if ip link show "$IFACE" | grep -q 'state UP\|LOWER_UP'; then
        log "Link is UP on $IFACE"
        break
    fi
    sleep 1
done

# ── Ping test ─────────────────────────────────────────────────────────────────
log "Pinging Raspberry Pi at $RPI_IP (3 packets)..."
if ping -c 3 -W 2 -I "$IFACE" "$RPI_IP" > /dev/null 2>&1; then
    log "SUCCESS: Raspberry Pi ($RPI_IP) is reachable"
else
    log "WARNING: No ping reply from $RPI_IP"
    log "  Possible causes:"
    log "    - Raspberry Pi not yet configured (run configure_rpi_network.sh on RPi)"
    log "    - Cable not connected"
    log "    - Wrong interface selected"
fi

# ── Persist with netplan (Ubuntu) ─────────────────────────────────────────────
NETPLAN_FILE="/etc/netplan/99-anubix-link.yaml"
if command -v netplan &>/dev/null; then
    log ""
    log "Writing netplan config for persistence across reboots..."
    cat > "$NETPLAN_FILE" << NETPLAN
network:
  version: 2
  ethernets:
    ${IFACE}:
      dhcp4: false
      addresses:
        - ${JETSON_IP}/${PREFIX}
      optional: true
NETPLAN
    netplan apply 2>/dev/null || log "  WARNING: netplan apply failed — config written but not applied"
    log "  Persistent config: $NETPLAN_FILE"
else
    log "  netplan not found — configuration is temporary (lost on reboot)."
    log "  To persist, add to /etc/network/interfaces or your distro's equivalent."
fi

log ""
log "==================================================="
log "  Jetson network configured"
log "  Interface : $IFACE"
log "  Jetson IP : $JETSON_IP / $PREFIX"
log "  RPi IP    : $RPI_IP (expected)"
log "==================================================="
log ""
log "  Also update the DDS config interface name:"
log "    nano ~/.ros/jetson_cyclone.xml"
log "  → change: <NetworkInterface name=\"eth0\"> to <NetworkInterface name=\"${IFACE}\">"
