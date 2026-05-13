#!/usr/bin/env python3
"""
ANUBIX Spectrometer Driver — Si-NIR sensor over TCP/IP
=======================================================
Implements the Si-NIR Communication Service rev 1 wire protocol over the
sensor's USB-Ethernet gadget (default 192.168.137.2, write=5000, read=5001),
then layers the bench-tested dark-current subtraction + crop-health analysis
on top of it.

Supported task types:
  - water_stress:    NDVI-based water stress index from reflectance
  - disease:         Spectral signature matching for plant disease
  - harvest_status:  Chlorophyll/carotenoid ratio for ripeness

Si-NIR wire protocol (see Si-NIR_Communication_Service_r1.pdf):
  TX command   = u32 length=192 + 48 × i32 little-endian fields
                 (operation, resolution, mode, zeroPadding, scanTime,
                  commonWavNum, opticalGain, apodizationSel,
                  GeneralData[40])
  RX runPSD    = u32 status + u32 length
                 + i64 PSD[4096] + i64 Wavenumber[4096]
                 PSD /= 2**33,   Wavenumber /= 2**30
The user reports minor doc inaccuracies, so the driver logs full hex dumps
of every TX/RX and also reports the first PSD samples interpreted as BOTH
int64/2**33 (per spec) and float64 (so a wrong-type guess is visible).

Usage standalone:
    python3 spectrometer_driver.py --task water_stress
    python3 spectrometer_driver.py --host 192.168.137.2 --task disease
"""

import os
import csv
import time
import socket
import struct
import logging
import argparse
import threading
import numpy as np
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, List

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
    raw_psd: np.ndarray
    corrected_psd: np.ndarray
    timestamp: float


@dataclass
class AnalysisResult:
    task_type: str
    value: float
    classification: str
    confidence: float
    details: dict


# ─────────────────────────────────────────────────────────────────────────────
# Background calibration
# ─────────────────────────────────────────────────────────────────────────────

class BackgroundCalibration:
    """Loads and manages the background/dark reference spectrum from bg.csv."""

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
        log.info(f"[CAL] Loaded background: {len(self.wavelengths)} channels, "
                 f"range [{self.wavelengths[0]:.1f} - {self.wavelengths[-1]:.1f}]")

    def subtract(self, raw_psd: np.ndarray) -> np.ndarray:
        """Subtract background from raw reading. Clips negative values to 0."""
        if len(raw_psd) != len(self.background_psd):
            log.warning(f"[CAL] Channel mismatch: raw={len(raw_psd)}, "
                        f"bg={len(self.background_psd)}. Interpolating.")
            from scipy.interpolate import interp1d
            interp = interp1d(
                np.linspace(0, 1, len(self.background_psd)),
                self.background_psd,
                kind='linear')
            bg_resampled = interp(np.linspace(0, 1, len(raw_psd)))
            corrected = raw_psd - bg_resampled
        else:
            corrected = raw_psd - self.background_psd
        return np.clip(corrected, 0.0, None)

    @property
    def num_channels(self) -> int:
        return len(self.wavelengths)


# ─────────────────────────────────────────────────────────────────────────────
# Spectrometer hardware interface
# ─────────────────────────────────────────────────────────────────────────────

class SpectrometerDevice:
    """
    Interface to the Si-NIR sensor over TCP/IP (spec rev 1).

    The sensor presents itself as a USB-Ethernet gadget at 192.168.137.2
    with two TCP ports:
        write_port = 5000  — host → sensor (binary command packets)
        read_port  = 5001  — sensor → host (binary response packets)

    Command packet:
        [u32 length=192][48 × i32 LE fields]
        fields = operation, resolution, mode, zeroPadding, scanTime,
                 commonWavNum, opticalGain, apodizationSel, GeneralData[40]

    runPSD (op 3) response:
        [u32 status][u32 length]
        [i64 PSD × 4096][i64 Wavenumber × 4096]
        PSD /= 2**33,  Wavenumber /= 2**30
    """

    # Operation codes (spec §2.2)
    OP_READ_MODULE_ID = 1
    OP_CHECK_BOARD = 2
    OP_RUN_PSD = 3
    OP_RUN_BACKGROUND = 4
    OP_RUN_SPECTRUM = 5
    OP_READ_SW_VERSION = 14

    PACKET_NUM_INTS = 48
    GENERAL_DATA_LEN = 40

    PSD_BUFFER_SAMPLES = 4096
    PSD_BUFFER_BYTES = PSD_BUFFER_SAMPLES * 8

    PSD_DEQUANT = float(1 << 33)
    WN_DEQUANT = float(1 << 30)

    # commonWavNum → point count (spec §2.1)
    COMMON_WAV_POINTS = {0: 0, 1: 65, 2: 129, 3: 257, 4: 513,
                         5: 1024, 6: 2048, 7: 4096}

    def __init__(self,
                 host: str = '192.168.137.2',
                 read_port: int = 5001,
                 write_port: int = 5000,
                 num_channels: int = 257,
                 integration_time_ms: int = 100,
                 zero_padding: int = 2,
                 optical_gain: int = 0,
                 apodization: int = 2,
                 connect_timeout_s: float = 5.0,
                 read_timeout_s: float = 15.0,
                 verbose_debug: bool = True):
        self.host = host
        self.read_port = int(read_port)
        self.write_port = int(write_port)
        self.num_channels = int(num_channels)
        self.integration_time_ms = int(integration_time_ms)
        self.zero_padding = int(zero_padding)
        self.optical_gain = int(optical_gain)
        self.apodization = int(apodization)
        self.connect_timeout_s = float(connect_timeout_s)
        self.read_timeout_s = float(read_timeout_s)
        self.verbose = bool(verbose_debug)

        # scanTime allowed range per spec §2.1: 10..224 ms
        self._scan_time_ms = max(10, min(224, self.integration_time_ms))
        if self._scan_time_ms != self.integration_time_ms:
            log.warning(
                f"[HW] scanTime clamped {self.integration_time_ms}"
                f"→{self._scan_time_ms} ms (Si-NIR allowed: 10..224)")

        # Pick commonWavNum that matches the configured channel count
        self._common_wav_num = next(
            (c for c, n in self.COMMON_WAV_POINTS.items()
             if n == self.num_channels),
            3,
        )
        chosen_n = self.COMMON_WAV_POINTS.get(self._common_wav_num)
        if chosen_n != self.num_channels:
            log.warning(
                f"[HW] num_channels={self.num_channels} has no exact "
                f"commonWavNum match; using commonWavNum="
                f"{self._common_wav_num} ({chosen_n} pts)")

        self._read_sock: Optional[socket.socket] = None
        self._write_sock: Optional[socket.socket] = None
        self._lock = threading.Lock()
        self.last_wavenumber: Optional[np.ndarray] = None

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
        """Open both sockets and best-effort identify the sensor."""
        # Open READ first — the sensor likely expects the read channel
        # ready before it processes a command on the write channel.
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

        # Probe operations. Log everything; do NOT fail connect on a probe
        # mismatch — the user warned the doc has minor inaccuracies, so we
        # want the per-operation truth in the terminal even if a probe is
        # slightly off.
        try:
            mid = self.read_module_id()
            log.info(f"[HW] Module ID: {mid!r}")
        except Exception as e:
            log.warning(f"[HW] readModuleID skipped: {e!r}")

        try:
            st = self.check_board()
            log.info(f"[HW] checkBoard status=0x{st:02x} "
                     f"({'READY' if st == 1 else 'NOT-READY/UNKNOWN'})")
        except Exception as e:
            log.warning(f"[HW] checkBoard skipped: {e!r}")

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
                      general_data: Optional[List[int]] = None) -> bytes:
        gd = list(general_data or [])
        gd = (gd + [0] * self.GENERAL_DATA_LEN)[:self.GENERAL_DATA_LEN]
        fields = [operation, resolution, mode, zero_padding, scan_time,
                  common_wav_num, optical_gain, apodization_sel] + gd
        data = struct.pack(f"<{self.PACKET_NUM_INTS}i", *fields)
        pkt = struct.pack("<I", len(data)) + data
        if self.verbose:
            log.info(
                f"[HW] TX op={operation} "
                f"(res={resolution} mode={mode} zp={zero_padding} "
                f"scan={scan_time}ms cwn={common_wav_num} "
                f"gain={optical_gain} apod={apodization_sel}) "
                f"datalen={len(data)} pkt={len(pkt)}B")
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

    # ────────────────── operations ──────────────────

    def check_board(self) -> int:
        """Op 2: returns 1 if sensor is ready, 0 otherwise (spec §2.2.3)."""
        with self._lock:
            self._send(self._build_packet(self.OP_CHECK_BOARD))
            resp = self._read_exact(1, label="checkBoard")
        return resp[0]

    def read_module_id(self) -> str:
        """Op 1: 21-byte null-terminated module ID string (spec §2.2.2)."""
        with self._lock:
            self._send(self._build_packet(self.OP_READ_MODULE_ID))
            resp = self._read_exact(21, label="readModuleID")
        end = resp.find(b'\x00')
        text = resp[:end] if end >= 0 else resp
        return text.decode('ascii', errors='replace').strip()

    def read_spectrum(self) -> np.ndarray:
        """Op 3 (runPSD): one scan, dequantized PSD as float64 (spec §2.2.4)."""
        if not self.is_connected:
            raise ConnectionError("Si-NIR not connected; call connect() first")

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
            log.info(f"[HW] runPSD sent; awaiting response "
                     f"(scanTime={self._scan_time_ms}ms, "
                     f"timeout={self.read_timeout_s:.1f}s)")

            header = self._read_exact(8, label="runPSD-header")
            status, length = struct.unpack("<II", header)
            log.info(f"[HW] runPSD header → status=0x{status:08x} "
                     f"length={length}")

            if status != 0:
                try:
                    self._drain_read_buffer(
                        max_bytes=self.PSD_BUFFER_BYTES * 2 + 1024,
                        timeout=1.0)
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

        if length <= 0 or length > self.PSD_BUFFER_SAMPLES:
            raise RuntimeError(
                f"Si-NIR returned implausible length={length} "
                f"(must be 1..{self.PSD_BUFFER_SAMPLES}); "
                f"raw header={header.hex()}")

        psd_q = np.frombuffer(psd_bytes, dtype='<i8')[:length]
        wn_q = np.frombuffer(wn_bytes, dtype='<i8')[:length]
        psd = psd_q.astype(np.float64) / self.PSD_DEQUANT
        wn = wn_q.astype(np.float64) / self.WN_DEQUANT
        self.last_wavenumber = wn

        log.info(f"[HW] runPSD parsed in {elapsed*1000:.0f}ms: "
                 f"{length} samples, "
                 f"PSD∈[{psd.min():.4e}, {psd.max():.4e}], "
                 f"WN∈[{wn.min():.1f}, {wn.max():.1f}] cm⁻¹")

        if self.verbose and length >= 3:
            # Spec claims int64/2**33 for PSD and int64/2**30 for WN.
            # User warned of minor doc issues — log the float64
            # reinterpretation too so the right type is obvious from one
            # terminal run (the version that gives sane numbers wins).
            mid = length // 2
            psd_dbl = np.frombuffer(psd_bytes, dtype='<f8')[:length]
            wn_dbl = np.frombuffer(wn_bytes, dtype='<f8')[:length]
            log.info(
                f"[HW] PSD[0,{mid},-1] as int64/2**33: "
                f"{psd[0]:.4e} {psd[mid]:.4e} {psd[-1]:.4e}")
            log.info(
                f"[HW] PSD[0,{mid},-1] as float64    : "
                f"{psd_dbl[0]:.4e} {psd_dbl[mid]:.4e} {psd_dbl[-1]:.4e}")
            log.info(
                f"[HW] WN [0,{mid},-1] as int64/2**30: "
                f"{wn[0]:.1f} {wn[mid]:.1f} {wn[-1]:.1f}")
            log.info(
                f"[HW] WN [0,{mid},-1] as float64    : "
                f"{wn_dbl[0]:.1f} {wn_dbl[mid]:.1f} {wn_dbl[-1]:.1f}")

        return psd


# ─────────────────────────────────────────────────────────────────────────────
# Spectral analysis engine
# ─────────────────────────────────────────────────────────────────────────────

class SpectralAnalyzer:
    """
    Processes calibrated spectra and classifies crop health.

    Implements three analysis modes:
      - water_stress:    Based on Water Band Index (WBI) and NDVI
      - disease:         Spectral angle mapping against disease signatures
      - harvest_status:  Chlorophyll/carotenoid ratio analysis
    """

    def __init__(self, wavelengths: np.ndarray):
        self.wavelengths = wavelengths
        self._wl_min = wavelengths[0]
        self._wl_max = wavelengths[-1]
        self._wl_step = (self._wl_max - self._wl_min) / (len(wavelengths) - 1)

    def _wl_to_idx(self, target_wl: float) -> int:
        """Convert wavelength to nearest channel index."""
        idx = int(round((target_wl - self._wl_min) / self._wl_step))
        return max(0, min(idx, len(self.wavelengths) - 1))

    def _band_mean(self, spectrum: np.ndarray, center_wl: float,
                   width: float = 20.0) -> float:
        """Mean reflectance in a band centered at center_wl with given width."""
        idx_lo = self._wl_to_idx(center_wl - width / 2)
        idx_hi = self._wl_to_idx(center_wl + width / 2)
        if idx_lo >= idx_hi:
            return float(spectrum[idx_lo])
        return float(np.mean(spectrum[idx_lo:idx_hi + 1]))

    def analyze(self, spectrum: np.ndarray, task_type: str) -> AnalysisResult:
        """Run analysis for the specified task type."""
        if task_type == 'water_stress':
            return self._analyze_water_stress(spectrum)
        elif task_type == 'disease':
            return self._analyze_disease(spectrum)
        elif task_type == 'harvest_status':
            return self._analyze_harvest(spectrum)
        else:
            log.warning(f"[ANALYSIS] Unknown task type: {task_type}")
            return AnalysisResult(
                task_type=task_type,
                value=0.0,
                classification='unknown',
                confidence=0.0,
                details={'error': f'unsupported task type: {task_type}'}
            )

    def _analyze_water_stress(self, spectrum: np.ndarray) -> AnalysisResult:
        """
        Water stress detection using spectral vegetation indices.

        Uses:
          - NDVI (Normalized Difference Vegetation Index)
          - WBI  (Water Band Index) if NIR range available
          - PRI  (Photochemical Reflectance Index)
        """
        # Compute indices based on available wavelength range
        # Map to relative positions in the spectrum
        n = len(spectrum)

        # Red region (~670nm equivalent) and NIR (~800nm equivalent)
        red_idx = int(n * 0.45)
        nir_idx = int(n * 0.65)

        red_band = float(np.mean(spectrum[max(0, red_idx - 3):red_idx + 3]))
        nir_band = float(np.mean(spectrum[max(0, nir_idx - 3):nir_idx + 3]))

        # NDVI
        if (nir_band + red_band) > 0:
            ndvi = (nir_band - red_band) / (nir_band + red_band)
        else:
            ndvi = 0.0

        # Green band for PRI approximation
        green_idx = int(n * 0.25)
        green_band = float(np.mean(spectrum[max(0, green_idx - 3):green_idx + 3]))

        # Water stress index (simplified)
        # Healthy vegetation: NDVI > 0.6
        # Mild stress: 0.3 < NDVI < 0.6
        # Severe stress: NDVI < 0.3
        if ndvi > 0.6:
            classification = 'healthy'
            stress_level = 0.0
        elif ndvi > 0.4:
            classification = 'mild_stress'
            stress_level = 0.4
        elif ndvi > 0.2:
            classification = 'moderate_stress'
            stress_level = 0.7
        else:
            classification = 'severe_stress'
            stress_level = 1.0

        confidence = min(1.0, abs(ndvi) * 1.5 + 0.3)

        return AnalysisResult(
            task_type='water_stress',
            value=stress_level,
            classification=classification,
            confidence=confidence,
            details={
                'ndvi': round(ndvi, 4),
                'red_reflectance': round(red_band, 4),
                'nir_reflectance': round(nir_band, 4),
                'green_reflectance': round(green_band, 4),
            }
        )

    def _analyze_disease(self, spectrum: np.ndarray) -> AnalysisResult:
        """
        Disease detection using spectral signature analysis.

        Compares the spectrum against known disease signatures using
        spectral angle mapping (SAM) and band ratio analysis.
        """
        n = len(spectrum)

        # Key diagnostic bands for plant disease:
        # - Blue absorption (chlorosis indicator)
        # - Red edge shift (stress indicator)
        # - Green reflectance peak changes
        blue_idx = int(n * 0.1)
        green_idx = int(n * 0.25)
        red_idx = int(n * 0.45)
        red_edge_idx = int(n * 0.55)
        nir_idx = int(n * 0.7)

        blue_band = float(np.mean(spectrum[max(0, blue_idx - 2):blue_idx + 2]))
        green_band = float(np.mean(spectrum[max(0, green_idx - 2):green_idx + 2]))
        red_band = float(np.mean(spectrum[max(0, red_idx - 2):red_idx + 2]))
        red_edge = float(np.mean(spectrum[max(0, red_edge_idx - 2):red_edge_idx + 2]))
        nir_band = float(np.mean(spectrum[max(0, nir_idx - 2):nir_idx + 2]))

        # Disease indicators:
        # 1. Red edge position shift (blue shift = stressed)
        # 2. Increased red reflectance (less chlorophyll absorption)
        # 3. Decreased NIR reflectance (cell structure damage)
        red_edge_ratio = red_edge / (red_band + 1e-6)
        nir_red_ratio = nir_band / (red_band + 1e-6)
        chlorophyll_index = (nir_band - red_edge) / (nir_band + red_edge + 1e-6)

        # Classification thresholds
        disease_score = 0.0
        if red_edge_ratio < 1.3:
            disease_score += 0.3
        if nir_red_ratio < 2.0:
            disease_score += 0.4
        if chlorophyll_index < 0.1:
            disease_score += 0.3

        if disease_score < 0.3:
            classification = 'healthy'
        elif disease_score < 0.6:
            classification = 'early_stage'
        else:
            classification = 'infected'

        confidence = 0.5 + disease_score * 0.4

        return AnalysisResult(
            task_type='disease',
            value=disease_score,
            classification=classification,
            confidence=confidence,
            details={
                'red_edge_ratio': round(red_edge_ratio, 4),
                'nir_red_ratio': round(nir_red_ratio, 4),
                'chlorophyll_index': round(chlorophyll_index, 4),
                'disease_score': round(disease_score, 4),
            }
        )

    def _analyze_harvest(self, spectrum: np.ndarray) -> AnalysisResult:
        """
        Harvest readiness assessment.

        Uses chlorophyll degradation and carotenoid accumulation
        as indicators of fruit/crop ripeness.
        """
        n = len(spectrum)

        # Relevant bands:
        # - Blue/violet: carotenoid absorption
        # - Green: chlorophyll reflectance
        # - Red: chlorophyll absorption
        # - Red edge: structural change indicator
        blue_idx = int(n * 0.1)
        green_idx = int(n * 0.25)
        red_idx = int(n * 0.45)
        nir_idx = int(n * 0.65)

        blue_band = float(np.mean(spectrum[max(0, blue_idx - 2):blue_idx + 2]))
        green_band = float(np.mean(spectrum[max(0, green_idx - 2):green_idx + 2]))
        red_band = float(np.mean(spectrum[max(0, red_idx - 2):red_idx + 2]))
        nir_band = float(np.mean(spectrum[max(0, nir_idx - 2):nir_idx + 2]))

        # Ripeness indicators:
        # As fruit ripens: chlorophyll decreases (green goes down, red goes up)
        # Carotenoids increase (blue absorption changes)
        green_red_ratio = green_band / (red_band + 1e-6)
        ndvi = (nir_band - red_band) / (nir_band + red_band + 1e-6)

        # Ripeness score: low green/red ratio + lower NDVI = more ripe
        ripeness_score = 1.0 - (green_red_ratio * 0.5 + ndvi * 0.3)
        ripeness_score = max(0.0, min(1.0, ripeness_score))

        if ripeness_score > 0.7:
            classification = 'ready'
        elif ripeness_score > 0.4:
            classification = 'nearly_ready'
        else:
            classification = 'not_ready'

        confidence = 0.6 + abs(ripeness_score - 0.5) * 0.6

        return AnalysisResult(
            task_type='harvest_status',
            value=ripeness_score,
            classification=classification,
            confidence=confidence,
            details={
                'green_red_ratio': round(green_red_ratio, 4),
                'ndvi': round(ndvi, 4),
                'ripeness_score': round(ripeness_score, 4),
            }
        )


# ─────────────────────────────────────────────────────────────────────────────
# Main spectrometer pipeline
# ─────────────────────────────────────────────────────────────────────────────

class SpectrometerPipeline:
    """
    Complete spectrometer pipeline: acquire -> calibrate -> analyze -> report.

    Stages published as status:
      1. "reading"      - Acquiring data from sensor
      2. "applying_ML"  - Running spectral analysis
      3. "uploading"    - Preparing results (or uploading to cloud)
      4. "success"      - Pipeline complete with results
      5. "failure"      - Error during any stage
    """

    def __init__(self, bg_path: str, device: 'SpectrometerDevice'):
        if device is None:
            raise ValueError(
                "SpectrometerPipeline requires a SpectrometerDevice instance")
        self.calibration = BackgroundCalibration(bg_path)
        self.device = device
        self.analyzer = SpectralAnalyzer(self.calibration.wavelengths)
        self._status_callback = None
        self._last_reading: Optional[SpectralReading] = None
        self._last_result: Optional[AnalysisResult] = None

    def set_status_callback(self, callback):
        """Set callback(status_string) for stage updates."""
        self._status_callback = callback

    def _emit_status(self, status: str):
        log.info(f"[PIPELINE] Stage: {status}")
        if self._status_callback:
            self._status_callback(status)

    def connect(self):
        """Connect to spectrometer hardware."""
        self.device.connect()

    def disconnect(self):
        """Disconnect from spectrometer hardware."""
        self.device.disconnect()

    def run(self, task_type: str) -> Tuple[str, Optional[AnalysisResult]]:
        """
        Execute the full spectrometer pipeline for the given task.

        Returns:
            (final_status, result) where final_status is 'success' or 'failure'
        """
        try:
            # Stage 1: Reading
            self._emit_status('reading')
            if not self.device.is_connected:
                self.device.connect()
            raw_psd = self.device.read_spectrum()
            timestamp = time.time()
            log.info(f"[PIPELINE] Acquired {len(raw_psd)} channels")

            # Stage 2: Background subtraction + ML analysis
            self._emit_status('applying_ML')
            corrected_psd = self.calibration.subtract(raw_psd)

            self._last_reading = SpectralReading(
                wavelengths=self.calibration.wavelengths,
                raw_psd=raw_psd,
                corrected_psd=corrected_psd,
                timestamp=timestamp,
            )

            result = self.analyzer.analyze(corrected_psd, task_type)
            self._last_result = result
            log.info(f"[PIPELINE] Analysis: {result.classification} "
                     f"(confidence={result.confidence:.2f})")

            # Stage 3: Upload / finalize
            self._emit_status('uploading')
            # In production: upload results to cloud backend here
            time.sleep(0.1)  # Placeholder for upload latency

            # Stage 4: Success
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
    parser.add_argument('--read-port', type=int, default=5001,
                        help='TCP port for inbound data (spec: 5001)')
    parser.add_argument('--write-port', type=int, default=5000,
                        help='TCP port for outbound commands (spec: 5000)')
    parser.add_argument('--integration', type=int, default=100,
                        help='scanTime in ms (10..224)')
    parser.add_argument('--zero-padding', type=int, default=2,
                        choices=[1, 2, 3],
                        help='FFT points (1=8k, 2=16k, 3=32k)')
    parser.add_argument('--optical-gain', type=int, default=0,
                        choices=[0, 1, 2],
                        help='0=saved on sensor, 1=calculated, 2=external')
    parser.add_argument('--apodization', type=int, default=2,
                        choices=[0, 1, 2, 3],
                        help='0=Boxcar 1=Gaussian 2=Happ-Genzel 3=Lorenz')
    parser.add_argument('--task', required=True,
                        choices=['water_stress', 'disease', 'harvest_status'],
                        help='Analysis task type')
    parser.add_argument('--bg', default=None,
                        help='Path to background CSV (default: bg.csv next to this script)')
    parser.add_argument('--repeat', type=int, default=1,
                        help='Number of readings to take')
    parser.add_argument('--interval', type=float, default=2.0,
                        help='Interval between repeated readings (seconds)')
    parser.add_argument('--quiet', action='store_true',
                        help='Suppress hex-dump debug logs')
    args = parser.parse_args()

    # Find bg.csv
    if args.bg:
        bg_path = args.bg
    else:
        bg_path = str(Path(__file__).parent / 'bg.csv')

    if not os.path.exists(bg_path):
        print(f"ERROR: Background file not found: {bg_path}")
        return 1

    # Create device + pipeline
    device = SpectrometerDevice(
        host=args.host,
        read_port=args.read_port,
        write_port=args.write_port,
        num_channels=257,
        integration_time_ms=args.integration,
        zero_padding=args.zero_padding,
        optical_gain=args.optical_gain,
        apodization=args.apodization,
        verbose_debug=not args.quiet,
    )

    pipeline = SpectrometerPipeline(
        bg_path=bg_path,
        device=device,
    )

    def status_cb(status):
        print(f"  STATUS: {status}")

    pipeline.set_status_callback(status_cb)

    print(f"{'='*50}")
    print(f"  ANUBIX Spectrometer - {args.task}")
    print(f"  Host: {args.host} (read={args.read_port}, write={args.write_port})")
    print(f"{'='*50}")

    pipeline.connect()

    try:
        for i in range(args.repeat):
            if args.repeat > 1:
                print(f"\n--- Reading {i+1}/{args.repeat} ---")

            status, result = pipeline.run(args.task)

            if result:
                print(f"\n  Result:")
                print(f"    Classification: {result.classification}")
                print(f"    Confidence:     {result.confidence:.2%}")
                print(f"    Value:          {result.value:.4f}")
                print(f"    Details:        {result.details}")
            else:
                print(f"\n  FAILED: pipeline returned no result")

            if i < args.repeat - 1:
                time.sleep(args.interval)
    finally:
        pipeline.disconnect()

    return 0


if __name__ == '__main__':
    exit(main())
