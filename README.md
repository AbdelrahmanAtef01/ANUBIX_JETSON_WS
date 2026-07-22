<p align="center">
  <img src="docs/assets/anubix_banner.png" alt="ANUBIX Banner" width="100%"/>
</p>

<h1 align="center">ANUBIX</h1>

<h3 align="center">
  Autonomous AI-Driven Agricultural Robot for Precision Plant Diagnostics
</h3>

<p align="center">
  <em>Graduation Project &mdash; Faculty of Computers and Artificial Intelligence, Cairo University (2025&ndash;2026)</em>
</p>

<p align="center">
  <a href="#architecture"><strong>Architecture</strong></a> &bull;
  <a href="#ros-2-packages"><strong>Packages</strong></a> &bull;
  <a href="#hardware"><strong>Hardware</strong></a> &bull;
  <a href="#getting-started"><strong>Getting Started</strong></a> &bull;
  <a href="#demo-videos"><strong>Demos</strong></a> &bull;
  <a href="#team"><strong>Team</strong></a>
</p>

<p align="center">
  <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS_2-Humble-blue?style=for-the-badge&logo=ros"/>
  <img alt="Python 3.10+" src="https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white"/>
  <img alt="Jetson Orin Nano" src="https://img.shields.io/badge/Jetson-Orin_Nano-76B900?style=for-the-badge&logo=nvidia&logoColor=white"/>
  <img alt="YOLO v8" src="https://img.shields.io/badge/YOLO-v8_Segmentation-FF6F00?style=for-the-badge"/>
  <img alt="License" src="https://img.shields.io/badge/License-Academic-lightgrey?style=for-the-badge"/>
</p>

---

## Overview

**ANUBIX** is a fully autonomous agricultural robot that navigates to crop locations, visually identifies diseased or stressed leaves using AI-powered computer vision, physically samples them with a robotic arm and gripper, performs near-infrared spectral analysis, and uploads diagnostic results to the cloud &mdash; all orchestrated by a natural-language AI agent that a farmer can command from a web interface.

The system combines **9 ROS 2 packages** running across a Jetson Orin Nano and Raspberry Pi, coordinated through a multi-subsystem supervisor architecture with real-time feedback loops, emergency stop propagation, and automatic failure recovery.

### Key Capabilities

| Capability | Description |
|---|---|
| **AI Agent Control** | Natural-language commands via OmniLink web UI &rarr; structured tool calls &rarr; hardware execution |
| **Autonomous Navigation** | GPS/map-based navigation to specific crop coordinates with vision-standoff mode |
| **Dual-Camera Vision** | Camera 1 (Intel RealSense, 3D depth) for target identification + Camera 2 (USB flange-mounted) for precision parallax calibration |
| **YOLO Segmentation** | YOLOv8 instance segmentation distinguishes healthy vs. diseased leaves with intelligent target selection |
| **6-DOF Robotic Arm** | MyCobot Pro 450 Elite with DH-based forward kinematics, capsule self-collision checking, and multi-seed IK solving |
| **Adaptive Gripper** | Elephant Robotics myGripperF100 with position-monitoring leaf detection and multi-retry pick sequences |
| **NIR Spectroscopy** | Si-NIR sensor for molecular-level plant health analysis with remote ML inference |
| **Cloud Integration** | Supabase for data persistence + photo upload &bull; OmniLink for AI agent hosting |
| **Safety Systems** | Z-floor monitoring, force-stop propagation, hardware collision detection, pre-flight workspace validation |

---

## Architecture

```
                    ┌─────────────────────────────────────┐
                    │        OmniLink Web UI (Cloud)       │
                    │   User sends natural-language task    │
                    └──────────────┬───────────────────────┘
                                   │ HTTP POST (tool calls)
                                   ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    JETSON ORIN NANO                                   │
│                                                                      │
│  ┌──────────────┐    /supervisor/*     ┌──────────────────────────┐  │
│  │ Master Node  │──────────────────────│  Navigation Node         │  │
│  │ (AI Bridge)  │                      │  (Nav2 / Simulated)      │  │
│  │              │                      └──────────────────────────┘  │
│  │ HTTP Server  │    /supervisor/*     ┌──────────────────────────┐  │
│  │ :5055        │──────────────────────│  Vision Node             │  │
│  │              │                      │  YOLO + RealSense + USB  │  │
│  │ OmniLink     │                      │  TensorRT / CUDA         │  │
│  │ Agent Profile│                      └──────────────────────────┘  │
│  │              │    /supervisor/*     ┌──────────────────────────┐  │
│  │ Tool Callback│──────────────────────│  Arm Node                │  │
│  │ Handler      │                      │  Pro 450 + IK + Collision│  │
│  │              │                      └──────────────────────────┘  │
│  │              │    /supervisor/*     ┌──────────────────────────┐  │
│  │              │──────────────────────│  Gripper Node            │  │
│  │              │                      │  myGripperF100 RS485     │  │
│  │              │                      └──────────────────────────┘  │
│  │              │    /supervisor/*     ┌──────────────────────────┐  │
│  │              │──────────────────────│  Spectrometer Node       │  │
│  │              │                      │  Si-NIR TCP/IP + ML      │  │
│  └──────────────┘                      └──────────────────────────┘  │
│                                                                      │
│  ┌──────────────┐                      ┌──────────────────────────┐  │
│  │ Jetson Bridge│                      │  Supabase Uploader       │  │
│  │ (Link Monitor│                      │  Photos + Readings       │  │
│  └──────────────┘                      └──────────────────────────┘  │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
                    │
                    │ Ethernet (CycloneDDS)
                    ▼
┌──────────────────────────────────────────────────────────────────────┐
│                    RASPBERRY PI (Future)                              │
│                                                                      │
│  ┌──────────────┐    ┌──────────────┐    ┌────────────────────────┐  │
│  │  RPi Bridge  │    │  Nav2 Stack  │    │  Motor Controllers    │  │
│  │  (Heartbeat) │    │  (SLAM/LIDAR)│    │  (Drive Base)         │  │
│  └──────────────┘    └──────────────┘    └────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

### Communication Topology

All inter-node communication uses **ROS 2 Humble** topics with carefully configured QoS profiles:

| Topic Pattern | QoS | Purpose |
|---|---|---|
| `/supervisor/*` | RELIABLE + TRANSIENT_LOCAL | Commands from master to subsystems |
| `/nav/status` | RELIABLE + VOLATILE | Navigation feedback |
| `/perception/status` | RELIABLE + VOLATILE | Vision pipeline results |
| `/arm/arm_status` | RELIABLE + VOLATILE | Arm movement feedback |
| `/arm/gripper_status` | RELIABLE + VOLATILE | Gripper operation feedback |
| `/spectrometer/status` | RELIABLE + VOLATILE | Spectral analysis status |
| `/supervisor/force_stop` | RELIABLE + VOLATILE | Emergency stop (edge-triggered) |

---

## ROS 2 Packages

The workspace contains **9 packages**, each handling a distinct subsystem:

### `anubix_master` &mdash; AI Agent Bridge & Mission Orchestrator

The central coordinator. Runs an HTTP server (port 5055) that receives structured tool calls from the OmniLink AI agent and translates them into ROS 2 supervisor commands. Manages the 11-step execution sequence for each agricultural task:

1. Set robot/task context IDs
2. Navigate to target (vision standoff mode)
3. Initial perception (Camera 1, wide-angle)
4. Complete navigation (drive to target)
5. Move arm to initial position
6. Precision perception (Camera 2, telephoto)
7. Move arm to grip position
8. Grip the leaf
9. Spectrometer scan
10. Release
11. Retract arm to home

### `anubix_vision` &mdash; YOLO-Powered Leaf Detection

Dual-camera computer vision pipeline running on CUDA/TensorRT:

- **Camera 1 (Intel RealSense D400 series)**: 3D depth-based target localization with intelligent scoring that prefers unhealthy leaves in the top-third Y zone and left hemisphere
- **Camera 2 (USB flange-mounted)**: Two-phase parallax calibration &mdash; detects the closest leaf to the gripper, commands a 1cm arm calibration move, re-identifies the same leaf via nearest-centroid matching, and computes X/Y/Z offsets from pixel displacement

### `anubix_arm` &mdash; Robotic Arm Control

Full control stack for the **MyCobot Pro 450 Elite**:

- Modified DH forward kinematics (6-DOF)
- Capsule-based self-collision checking across all non-adjacent link pairs
- Multi-seed inverse kinematics with collision-aware solution selection
- Z-floor monitoring thread (emergency stop if Z breaches floor)
- Pre-flight workspace validation
- Direct Cartesian movement via `send_coords`

### `anubix_gripper` &mdash; Adaptive Leaf Gripper

Controls the **myGripperF100** via USB-RS485 (Modbus RTU):

- Position-monitoring leaf detection (ultra-sensitive, tracks stable readings)
- Multi-retry pick sequences with configurable torque and speed
- Dual interface: supervisor bridge (for master node) + direct topic/service control
- Auto-detection of USB serial port

### `anubix_spectrometer` &mdash; NIR Spectral Analysis

Interface to the **Si-NIR sensor** over dual TCP/IP sockets:

- 5-scan acquisition with mean averaging
- Background calibration (division-based, bg.csv reference)
- Remote ML inference server for disease classification
- Byte-exact reproduction of the reference `pyConnect` pipeline

### `anubix_navigation` &mdash; Autonomous Navigation

Navigation stack with vision-standoff capability:

- Configurable standoff distance for camera-first approach
- Preemption support (new goal cancels current navigation)
- Integration point for Nav2 action server

### `anubix_supabase` &mdash; Cloud Data Pipeline

Automated data upload triggered by successful spectrometer readings:

- Photo capture from USB camera
- Image upload to Supabase Storage
- Structured reading insertion (ReadingModel) with robot/task IDs
- Retry logic with configurable attempts

### `anubix_jetson_bridge` &mdash; Cross-Machine Link Monitor

Maintains the Jetson-RPi communication link:

- 1 Hz bidirectional heartbeat
- Connection loss detection with configurable timeout
- Command and feedback logging for traceability
- Statistics tracking

### `anubix_bringup` &mdash; System Launch

Single-command launch for the entire Jetson stack:

```bash
ros2 launch anubix_bringup jetson.launch.py                    # simulation mode
ros2 launch anubix_bringup jetson.launch.py simulate:=false     # hardware mode
```

---

## Hardware

### Bill of Materials

| Component | Model | Role | Interface |
|---|---|---|---|
| **Compute (Primary)** | NVIDIA Jetson Orin Nano | Vision inference, arm control, master node | &mdash; |
| **Compute (Navigation)** | Raspberry Pi 4 | Nav2, motor control, LIDAR processing | Ethernet to Jetson |
| **Robotic Arm** | MyCobot Pro 450 Elite | 6-DOF manipulation, leaf sampling | Ethernet (TCP 4500) |
| **Gripper** | Elephant Robotics myGripperF100 | Leaf gripping with force sensing | USB-RS485 (Modbus RTU) |
| **Depth Camera** | Intel RealSense D400 series | 3D leaf localization (Camera 1) | USB 3.0 |
| **Flange Camera** | USB Webcam | Precision parallax calibration (Camera 2) | USB 2.0 |
| **Spectrometer** | Si-NIR Sensor | Near-infrared spectral analysis | USB-Ethernet (TCP 5000/5001) |
| **Drive Base** | Custom differential drive | Mobile platform navigation | RPi GPIO / CAN |

### Network Configuration

```
Jetson Orin Nano ─── 192.168.0.100  ──┐
MyCobot Pro 450  ─── 192.168.0.232  ──┤── Ethernet Switch
Raspberry Pi     ─── 192.168.10.2   ──┘
Si-NIR Sensor    ─── 192.168.137.2  ──── USB-Ethernet Gadget (direct)
```

---

## The 11-Step Execution Sequence

Each agricultural task follows a precise, AI-orchestrated sequence with built-in failure recovery:

```
  Step 1: Set Context ──────────────────── robot_id + task_id
       │
  Step 2: Navigate (Standoff) ──────────── stop 1m short, vision=true
       │
  Step 3: Initial Perception ───────────── Camera 1 (RealSense, 3D)
       │                                   YOLO segmentation → target leaf
       │
  Step 4: Navigate (Final) ─────────────── drive all the way, vision=false
       │
  Step 5: Arm → Initial Position ───────── move to Camera 1 target
       │
  Step 6: Precision Perception ─────────── Camera 2 (USB, parallax)
       │                                   re-identify leaf → XYZ offset
       │
  Step 7: Arm → Grip Position ─────────── move to refined target
       │
  Step 8: Grip ─────────────────────────── close gripper, detect leaf
       │                                   up to 5 retry attempts
       │
  Step 9: Spectrometer Scan ────────────── 5× NIR readings → mean → ML
       │                                   → disease classification
       │
  Step 10: Release ─────────────────────── open gripper gently
       │
  Step 11: Retract ─────────────────────── arm → home position
       │
  Step 12: Next Task or Go Home ────────── queue management
```

**Failure Recovery**: Every step has a defined recovery protocol &mdash; blocked navigation triggers retry then skip, perception failure on Camera 1 skips the plant, perception failure on Camera 2 continues (the arm is close enough), grip failure retries up to 3 times with re-perception, mechanical errors trigger emergency stop.

---

## Getting Started

### Prerequisites

- **NVIDIA Jetson Orin Nano** with JetPack 5.x+
- **ROS 2 Humble** (Ubuntu 22.04)
- **Python 3.10+**
- **CUDA** + **TensorRT** (for YOLO inference)

### Installation

```bash
# Clone the workspace
git clone https://github.com/AbdelrahmanAtef01/ANUBIX_JETSON_WS.git
cd ANUBIX_JETSON_WS

# Install Python dependencies
pip3 install omnilink pymycobot ultralytics opencv-python pyrealsense2 requests numpy supabase

# Build the ROS 2 workspace
colcon build --symlink-install
source install/setup.bash
```

### Export the YOLO Model for TensorRT

```bash
yolo export model=best.pt format=engine half=true
# Place the resulting best.engine in the workspace root
```

### Environment Variables

```bash
export OMNI_KEY="olink_YOUR_KEY_HERE"        # OmniLink API key
export SUPABASE_URL="https://your-project.supabase.co"
export SUPABASE_KEY="your-anon-key"
```

### Launch

```bash
# Simulation mode (no hardware required)
ros2 launch anubix_bringup jetson.launch.py

# Hardware mode (all devices connected)
ros2 launch anubix_bringup jetson.launch.py simulate:=false

# Individual nodes
ros2 run anubix_arm arm_node
ros2 run anubix_vision vision_node
ros2 run anubix_gripper gripper_node
ros2 run anubix_spectrometer spectrometer_node
```

### Using the AI Agent

1. Launch the system (master node starts the HTTP server on port 5055)
2. Set up an SSH tunnel if accessing remotely: `ssh -L 5055:localhost:5055 user@jetson-ip`
3. Open the **OmniLink Web UI** and select the **ANUBIX** agent
4. Send a task in natural language:
   ```
   Check for disease at coordinates (40, 45) using
   robot_id=34a957fd-d45c-4dbf-8e02-be8e1b5e349a
   task_id=40e4060b-5bc8-4044-9d71-046fee27a757
   ```
5. Watch the AI agent orchestrate the full 11-step sequence automatically

### Arm Mission Test

```bash
# Test all arm reachability points
ros2 run anubix_arm mission

# Test specific points (historically problematic collision zones)
ros2 run anubix_arm mission -- 3 6 7
```

---

## Demo Videos

### Camera 1 &mdash; RealSense Target Selection

The base-mounted Intel RealSense camera performs YOLO instance segmentation to identify and score leaves. The algorithm prefers unhealthy leaves (disease detection priority), applies Y-zone penalties, and uses hemisphere preference for optimal gripper approach.

https://github.com/user-attachments/assets/camera1_realsense_demo.mp4

> **Video**: `demo_output/camera1_realsense_demo.mp4` &mdash; Shows YOLO segmentation masks, hemisphere selection (left preferred), Y-zone penalty bands, per-leaf scoring breakdown, and target lock.

### Camera 2 &mdash; USB Flange Parallax Calibration

The flange-mounted USB camera performs a two-phase precision calibration. Phase 1 identifies the closest leaf to the gripper. The arm then moves 1cm right for calibration. Phase 2 re-identifies the same leaf and computes pixel-to-centimeter scale, XY offsets, and Z depth via vertical parallax disparity.

https://github.com/user-attachments/assets/camera2_usb_flange_demo.mp4

> **Video**: `demo_output/camera2_usb_flange_demo.mp4` &mdash; Shows Phase 1 closest-leaf detection, calibration transition, Phase 2 re-identification with parallax math overlay.

---

## Project Structure

```
anubix_ws/
├── src/
│   ├── anubix_master/           # AI agent bridge + mission orchestrator
│   │   ├── anubix_master/
│   │   │   ├── ros_master_node.py        # Tool callback HTTP server
│   │   │   └── command_parser.py         # Command parsing utilities
│   │   ├── config/
│   │   │   └── master_params.yaml
│   │   └── launch/
│   │       └── master.launch.py
│   │
│   ├── anubix_vision/           # YOLO leaf detection (CUDA/TensorRT)
│   │   ├── anubix_vision/
│   │   │   ├── vision_node.py            # Dual-camera pipeline
│   │   │   └── leaf_detection.py         # Pure-function detection helpers
│   │   ├── config/
│   │   │   └── vision_params.yaml
│   │   └── launch/
│   │       └── vision.launch.py
│   │
│   ├── anubix_arm/              # Pro 450 arm control + kinematics
│   │   ├── anubix_arm/
│   │   │   ├── arm_node.py               # Arm control node
│   │   │   └── mission.py                # Test runner
│   │   ├── config/
│   │   │   └── arm_params.yaml
│   │   └── launch/
│   │       └── arm.launch.py
│   │
│   ├── anubix_gripper/          # myGripperF100 control
│   │   ├── anubix_gripper/
│   │   │   ├── gripper_node.py           # Supervisor + direct interface
│   │   │   ├── elegripper.py             # Low-level Modbus driver
│   │   │   └── gripper_sender.py         # Manual command sender
│   │   ├── config/
│   │   │   └── gripper_params.yaml
│   │   └── launch/
│   │       └── (included via bringup)
│   │
│   ├── anubix_spectrometer/     # Si-NIR spectral analysis
│   │   ├── anubix_spectrometer/
│   │   │   ├── spectrometer_node.py      # ROS 2 node
│   │   │   └── spectrometer_driver.py    # Si-NIR TCP driver + ML client
│   │   ├── config/
│   │   │   ├── spectrometer_params.yaml
│   │   │   └── bg.csv                    # Background calibration data
│   │   └── launch/
│   │       └── spectrometer.launch.py
│   │
│   ├── anubix_navigation/       # Navigation stack
│   │   ├── anubix_navigation/
│   │   │   └── nav_node.py               # Vision-standoff navigation
│   │   ├── config/
│   │   │   └── nav_params.yaml
│   │   └── launch/
│   │       └── navigation.launch.py
│   │
│   ├── anubix_supabase/         # Cloud data pipeline
│   │   ├── anubix_supabase/
│   │   │   ├── supabase_node.py          # Upload orchestrator
│   │   │   └── supabase_uploader.py      # Supabase client wrapper
│   │   ├── config/
│   │   │   └── supabase_params.yaml
│   │   └── launch/
│   │       └── supabase.launch.py
│   │
│   ├── anubix_jetson_bridge/    # Cross-machine link monitor
│   │   ├── anubix_jetson_bridge/
│   │   │   └── jetson_bridge_node.py
│   │   ├── config/
│   │   │   └── jetson_bridge_params.yaml
│   │   └── launch/
│   │       └── jetson_bridge.launch.py
│   │
│   └── anubix_bringup/          # System launch files
│       └── launch/
│           └── jetson.launch.py          # Single-command full launch
│
├── agent_config/                # OmniLink AI agent configuration
│   ├── ANUBIX_AGENT_PROMPT_v3_TOOLCALLS.txt
│   ├── configure_anubix_agent.py
│   └── verify_configuration.py
│
├── demo_output/                 # Demo videos
│   ├── camera1_realsense_demo.mp4
│   └── camera2_usb_flange_demo.mp4
│
├── vision_demo.py               # Fully-annotated vision demo generator
├── TESTING.md                   # Stack-by-stack testing guide
└── README.md                   # This file
```

---

## Technical Deep Dives

### Self-Collision Avoidance (Arm)

The arm node implements a capsule-based self-collision checker using the Pro 450's Modified DH parameters. Each of the 6 links is modeled as a capsule (line segment + radius), and all non-adjacent pairs are checked for intersection:

- **Forward Kinematics**: Computes 7 joint-frame origins from 6 joint angles using the DH chain
- **Collision Check**: Minimum segment-segment distance across 9 non-adjacent link pairs
- **IK Selection**: When multiple inverse kinematics solutions exist, the one with the greatest minimum link clearance is preferred
- **Path Validation**: Joint-space interpolation with 20 intermediate samples to catch mid-path collisions

### Parallax Depth Estimation (Vision)

Camera 2's depth estimation uses a calibration-based parallax technique:

1. **Phase 1**: Identify the target leaf and record its centroid `(cx1, cy1)`
2. **Calibration Move**: Command the arm to move exactly 1cm in +X
3. **Phase 2**: Re-identify the same leaf at `(cx2, cy2)` using nearest-centroid matching
4. **Scale**: `pixels_per_cm = sqrt((cx2-cx1)^2 + (cy2-cy1)^2) / 1.0`
5. **XY Offset**: `dx_cm = (cx2 - gripper_x) / pixels_per_cm`
6. **Z Depth**: `depth_cm = (1.0 * pixels_per_cm) / vertical_shift_px`

### Spectrometer Pipeline

The spectral analysis follows the reference `pyConnect` pipeline byte-for-byte:

1. **Hardware Init**: `check_board` &rarr; `set_gain_settings` &rarr; `set_source_settings` &rarr; `read_module_id`
2. **Acquisition**: 5 scans via `run_psd` (Op 3), each dequantized as `PSD = (i64 / 2^33) * 100`
3. **Normalization**: `mean(5_scans) / bg.csv`, rounded to 8 decimals
4. **Classification**: POST 257-point feature vector to remote ML server
5. **Result**: `"Control with virus"` &rarr; `infected` | `"Control without virus"` &rarr; `healthy`

---

## Team

<table>
  <tr>
    <td align="center">
      <a href="https://github.com/AbdelrahmanAtef01">
        <img src="https://github.com/AbdelrahmanAtef01.png" width="100px;" alt=""/>
        <br />
        <sub><b>Abdelrahman Atef</b></sub>
      </a>
      <br />
      <sub>Software & System Integration Lead</sub>
      <br />
      <sub>Jetson Stack, AI Agent, Vision, Arm Control</sub>
    </td>
  </tr>
</table>

> **Supervised by**: Faculty of Computers and Artificial Intelligence, Cairo University

---

## Acknowledgments

- **Cairo University** &mdash; Faculty of Computers and Artificial Intelligence, for academic supervision and project guidance
- **NVIDIA** &mdash; Jetson Orin Nano platform and TensorRT acceleration
- **Elephant Robotics** &mdash; MyCobot Pro 450 Elite and myGripperF100 hardware
- **Intel** &mdash; RealSense depth camera SDK
- **Ultralytics** &mdash; YOLOv8 segmentation framework
- **OmniLink** &mdash; AI agent platform for natural-language robot control
- **Supabase** &mdash; Cloud database and storage infrastructure
- **Open Robotics** &mdash; ROS 2 Humble framework

---

## License

This project was developed as a graduation project at the Faculty of Computers and Artificial Intelligence, Cairo University. All rights are reserved to the project team and the university.

---

<p align="center">
  <sub>Built with determination, caffeine, and a lot of ROS 2 QoS debugging.</sub>
</p>
