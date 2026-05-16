#!/usr/bin/env python3
"""
Si-NIR framing probe.

The sensor accepts our TCP packets but never replies, then RSTs after a few
commands. That pattern means our wire-framing doesn't match what the sensor's
TCP service expects — the Si-NIR_Communication_Service_r1.pdf spec has
"minor inaccuracies" per the team's memory.

This script sends the smallest possible op (checkBoard, operation=2) under
every plausible framing variant and prints what comes back on each socket.
Whichever variant returns non-empty bytes is the correct framing — the
response content itself tells us byte-order, status size, and port roles.

Run on the Jetson (no rebuild needed):
    python3 sinir_probe.py

Paste the entire output back to the assistant.
"""

import socket
import struct
import time
import sys

HOST = '192.168.137.2'


def hexp(b: bytes, n: int = 48) -> str:
    if not b:
        return '(empty)'
    return b.hex() if len(b) <= n else f"{b[:n].hex()}…({len(b)}B)"


def io_dual(wport: int, rport: int, pkt: bytes, label: str, rx_to: float = 3.0):
    print(f"\n[{label}]  WRITE→{wport}  READ←{rport}")
    print(f"    TX {len(pkt)}B: {hexp(pkt)}")
    try:
        rs = socket.socket(); rs.settimeout(5); rs.connect((HOST, rport))
        ws = socket.socket(); ws.settimeout(5); ws.connect((HOST, wport))
        ws.sendall(pkt)
        rs.settimeout(rx_to)
        buf = bytearray()
        end = time.time() + rx_to
        while time.time() < end and len(buf) < 4096:
            try:
                c = rs.recv(4096)
                if not c:
                    break
                buf.extend(c)
                time.sleep(0.05)
            except socket.timeout:
                break
        print(f"    RX {len(buf)}B on read sock: {hexp(bytes(buf))}")
        ws.settimeout(0.4)
        try:
            extra = ws.recv(4096)
            if extra:
                print(f"    !!! UNEXPECTED on write sock: {hexp(extra)}")
        except (socket.timeout, OSError):
            pass
        rs.close()
        ws.close()
    except Exception as e:
        print(f"    ERROR: {e!r}")
    time.sleep(0.3)


def io_single(port: int, pkt: bytes, label: str, rx_to: float = 3.0):
    print(f"\n[{label}]  SINGLE SOCK on {port}")
    print(f"    TX {len(pkt)}B: {hexp(pkt)}")
    try:
        s = socket.socket(); s.settimeout(5); s.connect((HOST, port))
        s.sendall(pkt)
        s.settimeout(rx_to)
        buf = bytearray()
        end = time.time() + rx_to
        while time.time() < end and len(buf) < 4096:
            try:
                c = s.recv(4096)
                if not c:
                    break
                buf.extend(c)
                time.sleep(0.05)
            except socket.timeout:
                break
        print(f"    RX {len(buf)}B: {hexp(bytes(buf))}")
        s.close()
    except Exception as e:
        print(f"    ERROR: {e!r}")
    time.sleep(0.3)


def main() -> int:
    # Build checkBoard (operation=2) packet variants
    fields = [2] + [0] * 47
    data_le = struct.pack('<48i', *fields)
    data_be = struct.pack('>48i', *fields)
    pkt_lenLE_le = struct.pack('<I', len(data_le)) + data_le   # spec literal
    pkt_lenBE_be = struct.pack('>I', len(data_be)) + data_be   # all big-endian
    pkt_nolen_le = data_le                                     # no length prefix
    pkt_nolen_be = data_be

    print(f"=== Si-NIR framing probe @ {HOST} ===")
    io_dual(5000, 5001, pkt_lenLE_le,
            "A. PDF map  + LEN-LE + fields-LE  (current driver code)")
    io_dual(5001, 5000, pkt_lenLE_le,
            "B. INVERSE map + LEN-LE + fields-LE")
    io_dual(5000, 5001, pkt_nolen_le,
            "C. PDF map  + NO len + fields-LE")
    io_dual(5001, 5000, pkt_nolen_le,
            "D. INVERSE map + NO len + fields-LE")
    io_dual(5000, 5001, pkt_lenBE_be,
            "E. PDF map  + all BIG-endian")
    io_dual(5001, 5000, pkt_lenBE_be,
            "F. INVERSE map + all BIG-endian")
    io_dual(5000, 5001, pkt_nolen_be,
            "G. PDF map  + NO len + fields-BE")
    io_dual(5001, 5000, pkt_nolen_be,
            "H. INVERSE map + NO len + fields-BE")
    io_single(5000, pkt_lenLE_le, "I. SINGLE sock on 5000 (req+resp same)")
    io_single(5001, pkt_lenLE_le, "J. SINGLE sock on 5001 (req+resp same)")

    print("\n=== DONE ===")
    return 0


if __name__ == '__main__':
    sys.exit(main())
