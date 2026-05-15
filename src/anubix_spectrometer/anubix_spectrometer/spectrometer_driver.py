#!/usr/bin/env python3
"""
ANUBIX Spectrometer Driver — Si-NIR sensor over TCP/IP
=======================================================
Mirrors pyConnect (1).py byte-for-byte and math-for-math. The reference
script is the canonical pipeline that the remote ML server was trained
against; deviating from its parameters or post-processing makes the
model output garbage. Anything that diverges from pyConnect is a bug.

Reference flow (per acquisition):
  1) check_board                                                       (op 2)
  2) set_gain_settings   (defaults; sensor always replies 0)           (op 27)
  3) set_source_settings (lamp warm-up packed into calibrationWells)   (op 22)
  4) read_module_id      (informational)                               (op 1)
  5) For i in 1..5:
        run_psd                                                        (op 3)
          scanTime=2000, zeroPadding=POINTS32K(3),
          commonWaveNum=POINTS257(3), opticalGain=INTERNAL(0),
          apodization=BOXCAR(0).
        Dequantize:  PSD = (i64 / 2**33) * 100   (NOTE the *100)
                     WN  =  i64 / 2**30
  6) psd_mean = np.mean(np.vstack(all_5_psd), axis=0)
  7) bg = bg.csv['Psd']
     normalized_psd = np.round(psd_mean / bg, 8)
  8) POST {"features": normalized_psd.tolist()} to remote ML server.
     Parse result["prediction"] as the diagnosis string.
"""

import os
import csv
import time
import socket
import struct
import logging
import argparse
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import requests

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("spectrometer")


# ─────────────────────────────────────────────────────────────────────────────
# Data structures
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class SpectralReading:
    wavelengths: np.ndarray
    raw_psd_stack: np.ndarray
    psd_mean: np.ndarray
    normalized_psd: np.ndarray
    timestamp: float


@dataclass
class AnalysisResult:
    task_type: str
    value: float
    classification: str
    confidence: float
    details: dict


# ─────────────────────────────────────────────────────────────────────────────
# Background calibration (DIVISION-based, matches pyConnect)
# ─────────────────────────────────────────────────────────────────────────────

class BackgroundCalibration:
    """Loads bg.csv (the reference PSD baseline) and applies the reference
    normalization: `psd_mean / bg`, rounded to 8 decimals — same as
    pyConnect."""

    def __init__(self, bg_path: str):
        self.wavelengths: np.ndarray = np.array([])
        self.background_psd: np.ndarray = np.array([])
        self._load(bg_path)

    def _load(self, path: str):
        wavelengths = []
        psd_values = []
        with open(path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                wavelengths.append(float(row['Wavelength']))
                psd_values.append(float(row['Psd']))
        self.wavelengths = np.array(wavelengths)
        self.background_psd = np.array(psd_values)
        log.info(
            f"[CAL] Loaded background: {len(self.wavelengths)} channels, "
            f"WL range [{self.wavelengths[0]:.1f} - "
            f"{self.wavelengths[-1]:.1f}]")

    def normalize(self, psd_mean: np.ndarray) -> np.ndarray:
        """Divide mean PSD by background PSD, round to 8 decimals.
        Element-wise division — matches `normalized_psd = psd_mean /
        bg_psd_scaled` then `np.round(..., 8)` in pyConnect."""
        if len(psd_mean) != len(self.background_psd):
            raise ValueError(
                f"PSD length {len(psd_mean)} does not match bg.csv length "
                f"{len(self.background_psd)} — reference assumes 257-pt "
                f"alignment. Regenerate bg.csv with the same sensor "
                f"settings (commonWaveNum=POINTS257).")
        normalized = psd_mean / self.background_psd
        return np.round(normalized, 8)

    @property
    def num_channels(self) -> int:
        return len(self.wavelengths)


# ─────────────────────────────────────────────────────────────────────────────
# Spectrometer hardware interface — Si-NIR over TCP/IP
# ─────────────────────────────────────────────────────────────────────────────

class SpectrometerDevice:
    """
    Interface to the Si-NIR sensor over TCP/IP (spec rev 1).

    Wire framing (empirical, see sinir_probe.py + pyConnect.ResponsePacket):
        TX command = 48 × i32 little-endian fields              (192 bytes)
                     [operation, resolution, mode, zeroPadding,
                      scanTime, commonWaveNum, opticalGain,
                      apodizationSel, calibrationWells[40]]
                     NO length prefix on TX.
        RX         = u32 LE payload_len + payload_len bytes.
        runPSD     payload = u32 status + u32 length
                             + i64 PSD[4096] + i64 Wavenumber[4096]
                             PSD = (i64 / 2**33) * 100,
                             WN  =  i64 / 2**30.
    """

    # Operation codes (spec §2.2, also pyConnect constants)
    OP_READ_MODULE_ID = 1
    OP_CHECK_BOARD = 2
    OP_RUN_PSD = 3
    OP_RUN_BACKGROUND = 4
    OP_RUN_ABSORBANCE = 5
    OP_SET_SOURCE_SETTINGS = 22
    OP_SET_GAIN_SETTINGS = 27

    # Packet layout
    PACKET_NUM_INTS = 48
    CALIBRATION_WELLS_LEN = 40

    # runPSD response constants
    MAX_PSD_LENGTH = 4096
    PSD_BUFFER_BYTES = MAX_PSD_LENGTH * 8
    PSD_DEQUANT = float(1 << 33)
    WN_DEQUANT = float(1 << 30)
    PSD_POST_SCALE = 100.0  # pyConnect: `(psd / 2**33) * 100`

    # commonWaveNum → point count (spec §2.1)
    COMMON_WAV_POINTS = {0: 0, 1: 65, 2: 129, 3: 257, 4: 513,
                         5: 1024, 6: 2048, 7: 4096}

    # pyConnect.run_set_source_settings values — DO NOT change without
    # re-collecting bg.csv. The lamp warm-up profile is baked into the
    # baseline.
    SRC_LAMPS_COUNT = 2
    SRC_LAMPS_SELECT = 0
    SRC_T1 = 14
    SRC_DELTA_T = 2
    SRC_T2_C1 = 5
    SRC_T2_C2 = 35
    SRC_T2_MAX = 10

    def __init__(self,
                 host: str = '192.168.137.2',
                 read_port: int = 5001,
                 write_port: int = 5000,
                 num_channels: int = 257,
                 scan_time_ms: int = 2000,
                 zero_padding: int = 3,
                 optical_gain: int = 0,
                 apodization: int = 0,
                 connect_timeout_s: float = 5.0,
                 read_timeout_s: float = 15.0,
                 verbose_debug: bool = True):
        self.host = host
        self.read_port = int(read_port)
        self.write_port = int(write_port)
        self.num_channels = int(num_channels)
        # NOTE: pyConnect uses scan_time_seconds=2 → scanTime=2000 ms,
        # which is well past the spec's stated 10..224 range. The sensor
        # accepts it in practice and the reference relies on it, so we
        # do NOT clamp. If you want to clamp, regenerate bg.csv first.
        self._scan_time_ms = int(scan_time_ms)
        self.zero_padding = int(zero_padding)
        self.optical_gain = int(optical_gain)
        self.apodization = int(apodization)
        self.connect_timeout_s = float(connect_timeout_s)
        self.read_timeout_s = float(read_timeout_s)
        self.verbose = bool(verbose_debug)

        # Pick commonWaveNum that matches the configured channel count
        self._common_wav_num = next(
            (c for c, n in self.COMMON_WAV_POINTS.items()
             if n == self.num_channels),
            3,  # default to POINTS257
        )
        chosen_n = self.COMMON_WAV_POINTS.get(self._common_wav_num)
        if chosen_n != self.num_channels:
            log.warning(
                f"[HW] num_channels={self.num_channels} has no exact "
                f"commonWaveNum match; using commonWaveNum="
                f"{self._common_wav_num} ({chosen_n} pts) — bg.csv "
                f"must match.")

        self._read_sock: Optional[socket.socket] = None
        self._write_sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self.last_wavelength: Optional[np.ndarray] = None

    # ────────────────── socket lifecycle ──────────────────

    def _open_sock(self, port: int, label: str) -> socket.socket:
        log.info(f"[HW] Opening {label} socket → {self.host}:{port} "
                 f"(connect_timeout={self.connect_timeout_s}s)")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.connect_timeout_s)
        try:
            s.connect((self.host, port))
        except OSError as e:
            s.close()
            raise ConnectionError(
                f"Could not reach Si-NIR {label} port at "
                f"{self.host}:{port}: {e}") from e
        s.settimeout(self.read_timeout_s)
        try:
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        log.info(f"[HW] {label} socket connected")
        return s

    def connect(self):
        """Open both sockets and run the reference startup sequence:
        check_board → set_gain_settings → set_source_settings →
        read_module_id. pyConnect does these unconditionally on every
        boot; skipping any of them changes the lamp behaviour and the
        readings drift away from bg.csv."""
        # READ first — sensor expects the read channel ready before it
        # processes a command on the write channel.
        self._read_sock = self._open_sock(self.read_port, "READ")
        try:
            self._write_sock = self._open_sock(self.write_port, "WRITE")
        except ConnectionError:
            try:
                self._read_sock.close()
            finally:
                self._read_sock = None
            raise
        log.info(f"[HW] Si-NIR sockets up @ {self.host} "
                 f"(write={self.write_port}, read={self.read_port})")

        self._drain_read_buffer(max_bytes=256, timeout=0.3)

        # Startup sequence — match pyConnect order exactly.
        try:
            st = self.check_board()
            log.info(f"[HW] checkBoard → status_byte=0x{st:02x}")
        except Exception as e:
            log.warning(f"[HW] checkBoard skipped: {e!r}")

        try:
            self.set_gain_settings()
        except Exception as e:
            log.warning(f"[HW] setGainSettings skipped: {e!r}")

        try:
            self.set_source_settings()
        except Exception as e:
            log.warning(f"[HW] setSourceSettings skipped: {e!r}")

        try:
            mid = self.read_module_id()
            log.info(f"[HW] Module ID: {mid!r}")
        except Exception as e:
            log.warning(f"[HW] readModuleID skipped: {e!r}")

    def disconnect(self):
        for attr in ('_read_sock', '_write_sock'):
            s = getattr(self, attr)
            if s is None:
                continue
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                s.close()
            except OSError:
                pass
            setattr(self, attr, None)
        log.info("[HW] Si-NIR sockets closed")

    @property
    def is_connected(self) -> bool:
        return self._read_sock is not None and self._write_sock is not None

    # ────────────────── wire helpers ──────────────────

    @staticmethod
    def _hex(b: bytes, head: int = 32, tail: int = 8) -> str:
        if len(b) <= head + tail + 4:
            return b.hex()
        return f"{b[:head].hex()}…{b[-tail:].hex()} ({len(b)}B)"

    def _build_packet(self, operation: int,
                      resolution: int = 0, mode: int = 0,
                      zero_padding: int = 0, scan_time: int = 0,
                      common_wav_num: int = 0, optical_gain: int = 0,
                      apodization_sel: int = 0,
                      calibration_wells: Optional[List[int]] = None) -> bytes:
        cw = list(calibration_wells or [])
        cw = (cw + [0] * self.CALIBRATION_WELLS_LEN)[:self.CALIBRATION_WELLS_LEN]
        fields = [operation, resolution, mode, zero_padding, scan_time,
                  common_wav_num, optical_gain, apodization_sel] + cw
        # Reference packs as `<8I 40I` (UNSIGNED). Match exactly — all
        # our field values are non-negative, so signed vs unsigned
        # produces identical bytes today, but the reference is the
        # source of truth.
        pkt = struct.pack(f"<{self.PACKET_NUM_INTS}I", *fields)
        if self.verbose:
            log.info(
                f"[HW] TX op={operation} "
                f"(res={resolution} mode={mode} zp={zero_padding} "
                f"scan={scan_time}ms cwn={common_wav_num} "
                f"gain={optical_gain} apod={apodization_sel}) "
                f"pkt={len(pkt)}B")
            log.info(f"[HW] TX bytes: {self._hex(pkt)}")
        return pkt

    def _send(self, pkt: bytes):
        if self._write_sock is None:
            raise ConnectionError("Si-NIR write socket not connected")
        self._write_sock.sendall(pkt)

    def _read_exact(self, n: int, label: str = "data") -> bytes:
        if self._read_sock is None:
            raise ConnectionError("Si-NIR read socket not connected")
        buf = bytearray()
        deadline = time.time() + self.read_timeout_s
        while len(buf) < n:
            remaining = deadline - time.time()
            if remaining <= 0:
                raise TimeoutError(
                    f"Si-NIR {label} read timed out after "
                    f"{self.read_timeout_s:.1f}s: got {len(buf)}/{n}B. "
                    f"Partial: {self._hex(bytes(buf))}")
            try:
                self._read_sock.settimeout(min(remaining, 2.0))
                chunk = self._read_sock.recv(min(65536, n - len(buf)))
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError(
                    f"Si-NIR closed connection during {label}; "
                    f"got {len(buf)}/{n}B. "
                    f"Partial: {self._hex(bytes(buf))}")
            buf.extend(chunk)
        if self.verbose:
            log.info(f"[HW] RX {label} {n}B: {self._hex(bytes(buf))}")
        return bytes(buf)

    def _drain_read_buffer(self, max_bytes: int = 4096, timeout: float = 0.2):
        if self._read_sock is None:
            return
        old = self._read_sock.gettimeout()
        self._read_sock.settimeout(timeout)
        drained = bytearray()
        try:
            while len(drained) < max_bytes:
                chunk = self._read_sock.recv(
                    min(4096, max_bytes - len(drained)))
                if not chunk:
                    break
                drained.extend(chunk)
        except (socket.timeout, BlockingIOError, OSError):
            pass
        finally:
            try:
                self._read_sock.settimeout(
                    old if old is not None else self.read_timeout_s)
            except OSError:
                pass
        if drained:
            log.warning(f"[HW] Drained {len(drained)} stale bytes: "
                        f"{self._hex(bytes(drained))}")

    def _recv_response(self, label: str) -> bytes:
        """Read a framed response: [u32 LE payload_len][payload bytes]."""
        hdr = self._read_exact(4, label=f"{label}-resp-len")
        payload_len = struct.unpack("<I", hdr)[0]
        if self.verbose:
            log.info(f"[HW] RX {label} response_len={payload_len}")
        if payload_len == 0:
            return b''
        if payload_len > 200_000:
            log.warning(
                f"[HW] {label} response_len={payload_len} is unusually "
                f"large; raw header={hdr.hex()}")
        payload = self._read_exact(payload_len, label=f"{label}-payload")
        return payload

    # ────────────────── operations ──────────────────

    def check_board(self) -> int:
        """Op 2: 0=OK/connected (per spec), non-zero=error."""
        with self._lock:
            self._send(self._build_packet(self.OP_CHECK_BOARD))
            payload = self._recv_response("checkBoard")
        status = payload[0] if payload else 0xFF
        log.info(f"[HW] checkBoard payload={payload.hex()} "
                 f"status_byte=0x{status:02x}")
        return status

    def set_gain_settings(self) -> int:
        """Op 27: pyConnect sends an all-zero packet; sensor always
        replies 0. Just clears the response buffer so subsequent ops
        line up."""
        with self._lock:
            self._send(self._build_packet(self.OP_SET_GAIN_SETTINGS))
            payload = self._recv_response("setGainSettings")
        result = int.from_bytes(payload, 'little') if payload else 0
        log.info(f"[HW] setGainSettings result={result}")
        return result

    def set_source_settings(self) -> int:
        """Op 22: configure lamp warm-up. Reference packs three values
        into calibrationWells[0..2]:
            [0] = lampsCount | (lampsSelect << 8)            (2)
            [1] = t1         | (deltaT << 8)                 (526)
            [2] = t2C1       | (t2C2 << 8) | (t2max << 16)   (664325)
        These exact numbers were used when bg.csv was collected — do
        not change them."""
        cw = [0] * self.CALIBRATION_WELLS_LEN
        cw[0] = self.SRC_LAMPS_COUNT | (self.SRC_LAMPS_SELECT << 8)
        cw[1] = self.SRC_T1 | (self.SRC_DELTA_T << 8)
        cw[2] = (self.SRC_T2_C1 | (self.SRC_T2_C2 << 8)
                 | (self.SRC_T2_MAX << 16))
        with self._lock:
            self._send(self._build_packet(
                self.OP_SET_SOURCE_SETTINGS,
                calibration_wells=cw))
            payload = self._recv_response("setSourceSettings")
        result = int.from_bytes(payload, 'little') if payload else 0
        log.info(f"[HW] setSourceSettings result={result} "
                 f"(wells[0..2]={cw[0]},{cw[1]},{cw[2]})")
        return result

    def read_module_id(self) -> str:
        """Op 1: null-terminated module ID string."""
        with self._lock:
            self._send(self._build_packet(self.OP_READ_MODULE_ID))
            payload = self._recv_response("readModuleID")
        end = payload.find(b'\x00')
        text = payload[:end] if end >= 0 else payload
        return text.decode('ascii', errors='replace').strip()

    def read_spectrum(self) -> Tuple[np.ndarray, np.ndarray]:
        """Op 3 (runPSD): one scan. Returns (psd, wavelength), both
        dequantized exactly the way pyConnect does it:
            psd = (i64 / 2**33) * 100
            wn  =  i64 / 2**30
        """
        if not self.is_connected:
            raise ConnectionError(
                "Si-NIR not connected; call connect() first")

        pkt = self._build_packet(
            operation=self.OP_RUN_PSD,
            resolution=0,
            mode=0,
            zero_padding=self.zero_padding,
            scan_time=self._scan_time_ms,
            common_wav_num=self._common_wav_num,
            optical_gain=self.optical_gain,
            apodization_sel=self.apodization,
        )

        with self._lock:
            t0 = time.time()
            self._send(pkt)
            log.info(
                f"[HW] runPSD sent; awaiting response "
                f"(scanTime={self._scan_time_ms}ms, "
                f"timeout={self.read_timeout_s:.1f}s)")

            # Outer framing: [u32 resp_len][payload]
            # Payload: [u32 status][u32 length][PSD 4096×i64][WN 4096×i64]
            resp_hdr = self._read_exact(4, label="runPSD-resp-len")
            resp_len = struct.unpack("<I", resp_hdr)[0]
            log.info(f"[HW] runPSD response_len={resp_len} bytes")

            inner_hdr = self._read_exact(8, label="runPSD-status+length")
            status, length = struct.unpack("<II", inner_hdr)
            log.info(f"[HW] runPSD → status=0x{status:08x} "
                     f"data_length={length}")

            if status != 0:
                remaining = max(0, resp_len - 8)
                if remaining > 0:
                    try:
                        self._drain_read_buffer(
                            max_bytes=remaining, timeout=2.0)
                    except Exception:
                        pass
                raise RuntimeError(
                    f"Si-NIR runPSD returned error status {status} "
                    f"(0x{status:08x}); see spec §2.2.4")

            psd_bytes = self._read_exact(self.PSD_BUFFER_BYTES,
                                         label="runPSD-PSD")
            wn_bytes = self._read_exact(self.PSD_BUFFER_BYTES,
                                        label="runPSD-WN")
            elapsed = time.time() - t0

        if length <= 0 or length > self.MAX_PSD_LENGTH:
            log.warning(
                f"[HW] data_length={length} outside valid range; "
                f"falling back to num_channels={self.num_channels}")
            length = self.num_channels

        psd_q = np.frombuffer(psd_bytes, dtype='<i8')[:length]
        wn_q = np.frombuffer(wn_bytes, dtype='<i8')[:length]
        # ── pyConnect dequantization (EXACT) ──
        psd = (psd_q.astype(np.float64) / self.PSD_DEQUANT) * self.PSD_POST_SCALE
        wn = wn_q.astype(np.float64) / self.WN_DEQUANT
        self.last_wavelength = wn

        log.info(
            f"[HW] runPSD parsed in {elapsed*1000:.0f}ms: "
            f"{length} samples, "
            f"PSD range=[{psd.min():.4e}, {psd.max():.4e}], "
            f"WN range=[{wn.min():.1f}, {wn.max():.1f}]")

        return psd, wn


# ─────────────────────────────────────────────────────────────────────────────
# Remote ML client
# ─────────────────────────────────────────────────────────────────────────────

class RemoteMLClient:
    """POST a 257-point normalized PSD to the remote ML server and parse
    the diagnosis string. The server is the only ground truth — there
    is no local fallback classifier."""

    def __init__(self, server_url: str, timeout_s: float = 10.0):
        if not server_url:
            raise ValueError(
                "ml_server_url is empty — remote ML is mandatory, no "
                "local fallback. Set the ml_server_url parameter.")
        self.server_url = server_url
        self.timeout_s = float(timeout_s)

    def predict(self, features: np.ndarray) -> dict:
        payload = {"features": features.tolist()}
        log.info(
            f"[ML] POST {self.server_url} "
            f"features={len(payload['features'])}-pt "
            f"timeout={self.timeout_s:.1f}s")
        resp = requests.post(self.server_url, json=payload,
                             timeout=self.timeout_s)
        if resp.status_code != 200:
            raise RuntimeError(
                f"ML server HTTP {resp.status_code}: {resp.text[:200]!r}")
        result = resp.json()
        if result.get("status") != "success":
            raise RuntimeError(
                f"ML server returned non-success status: "
                f"{result.get('status')!r} message="
                f"{result.get('message')!r}")
        prediction = result.get("prediction")
        if prediction is None:
            raise RuntimeError(
                f"ML server response missing 'prediction' field: "
                f"{result!r}")
        log.info(f"[ML] prediction={prediction!r}")
        return result


# ─────────────────────────────────────────────────────────────────────────────
# Main spectrometer pipeline
# ─────────────────────────────────────────────────────────────────────────────

class SpectrometerPipeline:
    """
    Complete spectrometer pipeline: read 5 → mean → divide by bg →
    round → POST to remote ML → publish.

    Stages emitted as status:
        reading      — acquiring 5 scans
        applying_ML  — normalizing + POSTing to remote model
        uploading    — handoff to /spectrometer/result subscribers
        success | failure
    """

    DEFAULT_ML_URL = "http://16.171.254.109:5000/predict"
    DEFAULT_NUM_READS = 5

    def __init__(self,
                 bg_path: str,
                 device: SpectrometerDevice,
                 ml_server_url: str = DEFAULT_ML_URL,
                 ml_timeout_s: float = 10.0,
                 num_reads: int = DEFAULT_NUM_READS):
        if device is None:
            raise ValueError(
                "SpectrometerPipeline requires a SpectrometerDevice")
        self.calibration = BackgroundCalibration(bg_path)
        self.device = device
        self.ml_client = RemoteMLClient(ml_server_url, ml_timeout_s)
        self.num_reads = max(1, int(num_reads))
        self._status_callback = None
        self._last_reading: Optional[SpectralReading] = None
        self._last_result: Optional[AnalysisResult] = None

    def set_status_callback(self, callback):
        self._status_callback = callback

    def _emit_status(self, status: str):
        log.info(f"[PIPELINE] Stage: {status}")
        if self._status_callback:
            self._status_callback(status)

    def connect(self):
        self.device.connect()

    def disconnect(self):
        self.device.disconnect()

    # ── prediction → classification mapping ─────────────────────────────────
    # The remote ML server returns a free-form `prediction` string.
    # Current model outputs: "Control with virus" or "Control without virus"
    # (note: capital C in "Control", older pyConnect models used "With Virus" / "Healthy").
    # The supabase_uploader expects `classification ∈ {infected,
    # early_stage, healthy, ...}`, so we collapse the ML output to those
    # tokens here — any mention of "with virus" means infected; "without
    # virus" means healthy.
    @staticmethod
    def _classify(prediction: str) -> Tuple[str, float]:
        p = (prediction or '').strip()
        p_low = p.lower()
        # Check for "with virus" first (more specific) - handles any capitalization
        if 'with virus' in p_low:
            return 'infected', 1.0
        # Check for "without virus" - handles any capitalization
        if 'without virus' in p_low:
            return 'healthy', 0.0
        # Fallback: any mention of virus/disease/infected
        if 'virus' in p_low or 'disease' in p_low or 'infected' in p_low:
            return 'infected', 1.0
        return 'healthy', 0.0

    def run(self, task_type: str) -> Tuple[str, Optional[AnalysisResult]]:
        """Execute the reference pipeline once. Returns (status, result)."""
        try:
            # ─── Stage 1: 5-scan acquisition ─────────────────────────
            self._emit_status('reading')
            if not self.device.is_connected:
                self.device.connect()

            all_psd: List[np.ndarray] = []
            wavelength: Optional[np.ndarray] = None

            for i in range(1, self.num_reads + 1):
                log.info(
                    f"[PIPELINE] Reading {i}/{self.num_reads}...")
                psd_i, wn_i = self.device.read_spectrum()
                all_psd.append(psd_i)
                if wavelength is None:
                    wavelength = wn_i

            raw_stack = np.vstack(all_psd)
            psd_mean = np.mean(raw_stack, axis=0)
            log.info(
                f"[PIPELINE] Mean of {self.num_reads} scans: "
                f"PSD range=[{psd_mean.min():.4e}, {psd_mean.max():.4e}]")

            # ─── Stage 2: normalize + remote ML inference ─────────────
            self._emit_status('applying_ML')
            normalized = self.calibration.normalize(psd_mean)
            log.info(
                f"[PIPELINE] Normalized (mean/bg, rounded 8): "
                f"range=[{normalized.min():.8f}, "
                f"{normalized.max():.8f}]  "
                f"head={normalized[:3].tolist()}")

            timestamp = time.time()
            self._last_reading = SpectralReading(
                wavelengths=(wavelength
                             if wavelength is not None
                             else self.calibration.wavelengths),
                raw_psd_stack=raw_stack,
                psd_mean=psd_mean,
                normalized_psd=normalized,
                timestamp=timestamp,
            )

            ml_response = self.ml_client.predict(normalized)
            prediction = str(ml_response.get('prediction', ''))
            classification, value = self._classify(prediction)

            result = AnalysisResult(
                task_type=task_type,
                value=value,
                classification=classification,
                confidence=0.0,
                details={
                    'ml_prediction': prediction,
                    'ml_server': self.ml_client.server_url,
                    'num_reads': self.num_reads,
                    'normalized_head': [float(x) for x in normalized[:3]],
                },
            )
            self._last_result = result
            log.info(
                f"[PIPELINE] ML decision: prediction={prediction!r} → "
                f"classification={classification} value={value}")

            # ─── Stage 3: handoff ───────────────────────────────────
            self._emit_status('uploading')

            # ─── Stage 4: success ───────────────────────────────────
            self._emit_status('success')
            return 'success', result

        except Exception as e:
            log.error(f"[PIPELINE] Failed: {e}")
            self._emit_status('failure')
            return 'failure', None

    @property
    def last_reading(self) -> Optional[SpectralReading]:
        return self._last_reading

    @property
    def last_result(self) -> Optional[AnalysisResult]:
        return self._last_result


# ─────────────────────────────────────────────────────────────────────────────
# CLI for standalone testing
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='ANUBIX Spectrometer Driver')
    parser.add_argument('--host', default='192.168.137.2',
                        help='Si-NIR IP (USB-Ethernet gadget)')
    parser.add_argument('--read-port', type=int, default=5001)
    parser.add_argument('--write-port', type=int, default=5000)
    parser.add_argument('--scan-time-ms', type=int, default=2000,
                        help='scanTime (pyConnect uses 2000)')
    parser.add_argument('--zero-padding', type=int, default=3,
                        choices=[1, 2, 3],
                        help='1=8k, 2=16k, 3=32k (pyConnect: 3)')
    parser.add_argument('--optical-gain', type=int, default=0,
                        choices=[0, 1, 2])
    parser.add_argument('--apodization', type=int, default=0,
                        choices=[0, 1, 2, 3],
                        help='0=Boxcar (pyConnect)')
    parser.add_argument('--task', default='disease',
                        help='Analysis task type (passed through to result)')
    parser.add_argument('--bg', default=None)
    parser.add_argument('--num-reads', type=int, default=5)
    parser.add_argument('--ml-url',
                        default=SpectrometerPipeline.DEFAULT_ML_URL)
    parser.add_argument('--ml-timeout', type=float, default=10.0)
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--interval', type=float, default=2.0)
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()

    bg_path = args.bg or str(Path(__file__).parent / 'bg.csv')
    if not os.path.exists(bg_path):
        print(f"ERROR: Background file not found: {bg_path}")
        return 1

    device = SpectrometerDevice(
        host=args.host,
        read_port=args.read_port,
        write_port=args.write_port,
        num_channels=257,
        scan_time_ms=args.scan_time_ms,
        zero_padding=args.zero_padding,
        optical_gain=args.optical_gain,
        apodization=args.apodization,
        verbose_debug=not args.quiet,
    )

    pipeline = SpectrometerPipeline(
        bg_path=bg_path,
        device=device,
        ml_server_url=args.ml_url,
        ml_timeout_s=args.ml_timeout,
        num_reads=args.num_reads,
    )

    pipeline.set_status_callback(lambda s: print(f"  STATUS: {s}"))

    print(f"{'='*50}")
    print(f"  ANUBIX Spectrometer — task={args.task}")
    print(f"  Host: {args.host}  ML: {args.ml_url}")
    print(f"  num_reads={args.num_reads}  scanTime={args.scan_time_ms}ms")
    print(f"{'='*50}")

    pipeline.connect()
    try:
        for i in range(args.repeat):
            if args.repeat > 1:
                print(f"\n--- Run {i+1}/{args.repeat} ---")
            status, result = pipeline.run(args.task)
            if result:
                print(f"\n  Result:")
                print(f"    Prediction:     "
                      f"{result.details.get('ml_prediction')}")
                print(f"    Classification: {result.classification}")
                print(f"    Value:          {result.value}")
            else:
                print(f"\n  FAILED (status={status})")
            if i < args.repeat - 1:
                time.sleep(args.interval)
    finally:
        pipeline.disconnect()

    return 0


if __name__ == '__main__':
    exit(main())
