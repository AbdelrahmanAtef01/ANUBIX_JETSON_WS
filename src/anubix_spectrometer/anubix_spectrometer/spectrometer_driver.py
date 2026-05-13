#!/usr/bin/env python3
"""
ANUBIX Spectrometer Driver
============================
Handles communication with the physical spectrometer sensor,
background subtraction, and spectral analysis for crop health assessment.

Supported task types:
  - water_stress:    NDVI-based water stress index from reflectance
  - disease:         Spectral signature matching for plant disease
  - harvest_status:  Chlorophyll/carotenoid ratio for ripeness

Hardware: TCP/IP spectrometer over USB-Ethernet gadget
          (257 channels, ~3921-7407 wavelength range, default 192.168.137.2:5555)
Background calibration: bg.csv (dark/reference spectrum)

Usage standalone (testing without ROS):
    python3 spectrometer.py --host 192.168.137.2 --task water_stress
    python3 spectrometer.py --simulate --task disease
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
    Interface to the physical spectrometer over TCP/IP.

    The sensor exposes a USB Ethernet gadget at a fixed static address
    (default 192.168.137.2:5555). We open a TCP socket, send an ASCII
    command, and read back either a binary float32 block or an ASCII
    CSV line — same wire protocol as the previous serial driver.

    Use SimulatedSpectrometer for tests without hardware.
    """

    def __init__(self, host: str = '192.168.137.2', tcp_port: int = 5555,
                 num_channels: int = 257, protocol: str = 'binary',
                 integration_time_ms: int = 100, connect_timeout_s: float = 5.0,
                 read_timeout_s: float = 5.0):
        self.host = host
        self.tcp_port = tcp_port
        self.num_channels = num_channels
        self.protocol = protocol
        self.integration_time_ms = integration_time_ms
        self.connect_timeout_s = connect_timeout_s
        self.read_timeout_s = read_timeout_s
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()

    def connect(self):
        """Open TCP connection to the spectrometer."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.connect_timeout_s)
        try:
            sock.connect((self.host, self.tcp_port))
        except OSError as e:
            sock.close()
            raise ConnectionError(
                f"Could not reach spectrometer at {self.host}:{self.tcp_port}: {e}"
            ) from e
        sock.settimeout(self.read_timeout_s)
        # Disable Nagle so single-byte commands flush immediately.
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        self._sock = sock
        log.info(f"[HW] Connected to spectrometer at "
                 f"{self.host}:{self.tcp_port}")
        self._drain()
        self._send_integration_time()

    def disconnect(self):
        """Close TCP connection."""
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None
            log.info("[HW] Disconnected from spectrometer")

    def _drain(self):
        """Discard anything buffered in the socket before a fresh exchange."""
        if self._sock is None:
            return
        self._sock.settimeout(0.05)
        try:
            while True:
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
        except (socket.timeout, BlockingIOError, OSError):
            pass
        finally:
            self._sock.settimeout(self.read_timeout_s)

    def _send_integration_time(self):
        """Configure integration time on the sensor."""
        if self._sock is None:
            return
        with self._lock:
            cmd = f"INT:{self.integration_time_ms}\n".encode('ascii')
            self._sock.sendall(cmd)
            time.sleep(0.1)
            resp = self._read_line(timeout=1.0)
            if resp:
                log.info(f"[HW] Integration time set: {resp}")

    def read_spectrum(self) -> np.ndarray:
        """Trigger a reading and return raw PSD array."""
        if self._sock is None:
            raise ConnectionError("Spectrometer socket not connected")
        with self._lock:
            self._sock.sendall(b'READ\n')

            if self.protocol == 'binary':
                data = self._read_binary()
            else:
                data = self._read_ascii()

        return data

    def _read_exact(self, n: int) -> bytes:
        """Block until ``n`` bytes have been received."""
        buf = bytearray()
        start = time.time()
        while len(buf) < n:
            try:
                chunk = self._sock.recv(n - len(buf))
            except socket.timeout:
                if time.time() - start > self.read_timeout_s:
                    raise TimeoutError(
                        f"Spectrometer read timed out after "
                        f"{self.read_timeout_s:.1f}s "
                        f"({len(buf)}/{n} bytes)")
                continue
            if not chunk:
                raise ConnectionError(
                    "Spectrometer closed the connection mid-read")
            buf.extend(chunk)
        return bytes(buf)

    def _read_line(self, timeout: Optional[float] = None) -> str:
        """Read a single \\n-terminated ASCII line from the socket."""
        if timeout is not None:
            self._sock.settimeout(timeout)
        line = bytearray()
        try:
            while True:
                ch = self._sock.recv(1)
                if not ch:
                    break
                if ch == b'\n':
                    break
                line.extend(ch)
        except socket.timeout:
            pass
        finally:
            self._sock.settimeout(self.read_timeout_s)
        return line.decode('ascii', errors='replace').strip()

    def _read_binary(self) -> np.ndarray:
        """Read binary float32 data from device."""
        expected_bytes = self.num_channels * 4
        raw = self._read_exact(expected_bytes)
        return np.frombuffer(raw, dtype=np.float32)

    def _read_ascii(self) -> np.ndarray:
        """Read ASCII CSV line from device."""
        line = self._read_line()
        values = [float(v) for v in line.split(',') if v.strip()]
        return np.array(values)

    @property
    def is_connected(self) -> bool:
        return self._sock is not None


class SimulatedSpectrometer:
    """
    Simulated spectrometer for testing without hardware.
    Generates synthetic spectral data based on task type.
    """

    def __init__(self, num_channels: int = 257, noise_level: float = 0.02):
        self.num_channels = num_channels
        self.noise_level = noise_level
        self._connected = False

    def connect(self):
        self._connected = True
        log.info("[SIM] Simulated spectrometer connected")

    def disconnect(self):
        self._connected = False
        log.info("[SIM] Simulated spectrometer disconnected")

    def read_spectrum(self) -> np.ndarray:
        """Generate synthetic plant reflectance spectrum."""
        x = np.linspace(0, 1, self.num_channels)

        # Simulate typical vegetation reflectance:
        # Low in visible (absorbed), high around 700nm (red edge), plateau in NIR
        green_peak = 0.15 * np.exp(-((x - 0.25) ** 2) / 0.005)
        red_edge = 0.4 / (1 + np.exp(-30 * (x - 0.55)))
        nir_plateau = 0.35 * (1 / (1 + np.exp(-20 * (x - 0.6))))

        spectrum = 0.05 + green_peak + red_edge + nir_plateau
        noise = np.random.normal(0, self.noise_level, self.num_channels)
        spectrum += noise

        return np.clip(spectrum, 0.0, 1.0)

    @property
    def is_connected(self) -> bool:
        return self._connected


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

    def __init__(self, bg_path: str, device=None, simulate: bool = False):
        self.calibration = BackgroundCalibration(bg_path)

        if simulate or device is None:
            self.device = SimulatedSpectrometer(
                num_channels=self.calibration.num_channels)
        else:
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
                        help='Spectrometer IP (USB-Ethernet gadget)')
    parser.add_argument('--tcp-port', type=int, default=5555,
                        help='Spectrometer TCP port')
    parser.add_argument('--task', required=True,
                        choices=['water_stress', 'disease', 'harvest_status'],
                        help='Analysis task type')
    parser.add_argument('--simulate', action='store_true',
                        help='Use simulated spectrometer (no hardware)')
    parser.add_argument('--bg', default=None,
                        help='Path to background CSV (default: bg.csv next to this script)')
    parser.add_argument('--repeat', type=int, default=1,
                        help='Number of readings to take')
    parser.add_argument('--interval', type=float, default=2.0,
                        help='Interval between repeated readings (seconds)')
    args = parser.parse_args()

    # Find bg.csv
    if args.bg:
        bg_path = args.bg
    else:
        bg_path = str(Path(__file__).parent / 'bg.csv')

    if not os.path.exists(bg_path):
        print(f"ERROR: Background file not found: {bg_path}")
        return 1

    # Create device
    device = None
    if not args.simulate:
        device = SpectrometerDevice(
            host=args.host,
            tcp_port=args.tcp_port,
            num_channels=257,
        )

    # Create pipeline
    pipeline = SpectrometerPipeline(
        bg_path=bg_path,
        device=device,
        simulate=args.simulate,
    )

    def status_cb(status):
        print(f"  STATUS: {status}")

    pipeline.set_status_callback(status_cb)

    print(f"{'='*50}")
    print(f"  ANUBIX Spectrometer - {args.task}")
    print(f"  Mode: {'Simulated' if args.simulate else f'{args.host}:{args.tcp_port}'}")
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
