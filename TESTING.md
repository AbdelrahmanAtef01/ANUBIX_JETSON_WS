# ANUBIX — Complete Testing & Verification Guide

Run these tests in order after installing on both machines.

**System requirements:**
- **Raspberry Pi**: Raspberry Pi OS (Debian Bookworm/Trixie) 64-bit
- **Jetson**: JetPack 5.x / 6.x (Ubuntu 20.04 / 22.04)

---

## Phase 0: Pre-Launch Checks

### On Raspberry Pi (Raspberry Pi OS / Debian)

```bash
# 1. Verify environment is loaded
source ~/.bashrc
env | grep -E "ROS_|CYCLONE|RMW"

# Expected output:
# ROS_DOMAIN_ID=42
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CYCLONEDDS_URI=file:///home/pi/.ros/rpi_cyclone.xml
# ROS_LOCALHOST_ONLY=0

# 2. Check network
ip addr show eth0
# Should show: 192.168.10.2/24

# 3. Ping Jetson
ping -c 3 192.168.10.1
# Should get replies

# 4. Verify DDS config
cat ~/.ros/rpi_cyclone.xml | grep -A2 "NetworkInterface"
# Should show: <NetworkInterface name="eth0">
```

### On Jetson

```bash
# 1. Verify environment is loaded
source ~/.bashrc
env | grep -E "ROS_|CYCLONE|RMW|OMNI|SUPABASE"

# Expected output includes:
# ROS_DOMAIN_ID=42
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# OMNI_KEY=olink_4ekYIgHACfZaGlq6WJOgu59U
# SUPABASE_URL=https://bdkutmmrcjckaazzzspe.supabase.co
# SUPABASE_KEY=sb_publishable_VY6...

# 2. Check network (look for USB-C ethernet interface)
ip addr show
# Should show 192.168.10.1/24 on usb0/enx*/enp*

# 3. Ping RPi
ping -c 3 192.168.10.2
# Should get replies

# 4. Verify DDS config interface name matches
cat ~/.ros/jetson_cyclone.xml | grep -A2 "NetworkInterface"
# Should show your actual interface name (e.g., <NetworkInterface name="usb0">)

# 5. Check YOLO model exists
ls -lh ~/anubix_ws/best.engine
# Should show the TensorRT model file

# 6. Check USB camera (optional, for vision testing)
ls /dev/video*
# Should show /dev/video0 (or similar)
```

---

## Phase 1: Launch Both Systems

### Terminal 1 — Raspberry Pi

```bash
source ~/.bashrc
ros2 launch anubix_bringup_rpi rpi_full.launch.py
```

**Expected output:**
```
  ANUBIX Navigation Node - Raspberry Pi
  Mode: SIMULATE
  Nav delay: 5.0s
[NAV] Subscribed to /supervisor/nav_goal (PoseStamped, TRANSIENT_LOCAL)
[NAV] Publishing on /nav/status (String, RELIABLE)
[NAV] Ready and waiting for goals.
[anubix_rpi_bridge]: RPi bridge ready | publishing heartbeat at 1 Hz
[anubix_rpi_bridge]: Listening for Jetson heartbeat on /bridge/jetson_heartbeat
```

### Terminal 2 — Jetson

```bash
source ~/.bashrc
ros2 launch anubix_bringup jetson.launch.py
```

**Expected output:**
```
  ANUBIX ROS 2 Master Node - Jetson Orin Nano
  4-stack architecture: NAV | PERCEPTION | ARM | SPECTRO
  Listening on agent "ANUBIX" (memory at X msgs)
[POLL] Poll loop started

  ANUBIX Arm Control Node - Jetson Orin Nano
  Mode: SIMULATE
  Arm move delay: 2.0s   Grip delay: 1.0s
[ARM] Ready and waiting for commands.

  ANUBIX Spectrometer Node - Jetson Orin Nano
  Host: TCP 192.168.137.2 (read=5000, write=5001)
  Channels: 257
  BG file: /opt/ros/.../anubix_spectrometer/config/bg.csv
[SPECTRO] Subscribed to /supervisor/spectral_target (String)
[SPECTRO] Publishing on /spectrometer/status, /spectrometer/result
[SPECTRO] Ready and waiting for targets.

  ANUBIX Vision Node - Jetson Orin Nano
  Model: ../best.engine
  RealSense SDK: AVAILABLE              (or: NOT FOUND (camera 1 disabled))
  USB camera index: 0
  Confidence threshold: 0.5
  Detection: up to 30.0s @ 2.0 Hz
  Gripper pixel (cam2 flange): (-1, -1) (<0 = frame centre)
[VISION] YOLO model loaded successfully: ../best.engine
[VISION] Ready and waiting for goals.

[anubix_jetson_bridge]: Jetson bridge ready | publishing heartbeat at 1 Hz

  ANUBIX Supabase Uploader Node
  Listening on /spectrometer/result
  robot_id       = '34a957fd-...'
  task_id        = '40e4060b-...'
  photo capture  = ENABLED (camera index 0)
```

> The spectrometer line **no longer** says "mode=simulated" — simulation
> has been removed. If the host/ports are wrong it will fail to connect
> and log `[HW] Could not reach spectrometer ... at 192.168.137.2:5000`.

---

## Phase 2: Connectivity Tests

Open **Terminal 3 on Jetson** (or RPi, doesn't matter — topics are shared):

```bash
source ~/.bashrc

# 1. List all topics — should see topics from BOTH machines
ros2 topic list
```

**Expected topics (partial list):**
```
/supervisor/nav_goal
/supervisor/nav_vision
/supervisor/perception_goal
/supervisor/target_camera
/supervisor/arm_nav_goal
/supervisor/grip
/supervisor/spectral_target
/supervisor/force_stop
/arm/current_pose
/nav/status
/perception/status
/perception/target_pose
/arm/arm_status
/arm/gripper_status
/arm/touch_status
/spectrometer/status
/spectrometer/result
/supabase/upload_status
/bridge/jetson_heartbeat
/bridge/rpi_heartbeat
/bridge/connection_status
```

```bash
# 2. Check heartbeats — should see messages at 1 Hz from BOTH sides
ros2 topic echo /bridge/jetson_heartbeat --once
ros2 topic echo /bridge/rpi_heartbeat --once

# 3. Check connection status
ros2 topic echo /bridge/connection_status --once
# Should show: jetson_alive: true, rpi_alive: true

# 4. Monitor bridge logs (look for "ESTABLISHED" and no ERROR messages)
ros2 topic echo /rosout | grep -i bridge
```

**✅ If you see topics from both machines and heartbeats flowing → DDS link is working**

---

## Phase 3: Stack-by-Stack Manual Tests

All tests run from **Terminal 3** (either machine — topics are shared).
Each stack uses its own log prefix: `[NAV]`, `[ARM]`, `[SPECTRO]`,
`[VISION]`, `[SUPABASE]`.

### Test 1: Navigation Stack (runs on RPi)

The nav stack publishes a two-stage status: it acknowledges the goal
with `"navigating"` immediately, then `"point_reached"` (or
`"failure"`) once the simulated motion completes. Whether the robot
stops 1 m short of the goal is controlled by the **separate**
`/supervisor/nav_vision` topic, which is *latched* — set it before
sending the goal.

```bash
# 1a. (Optional) Tell nav to stop short of the goal so the camera can
#     take over. Default is False; the master flips this for you when
#     the OmniLink "navigate" tool fires with vision=true.
ros2 topic pub --once /supervisor/nav_vision std_msgs/Bool '{data: true}'

# 1b. Send a navigation goal
ros2 topic pub --once /supervisor/nav_goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 3.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}'
```

**Watch Terminal 1 (RPi):**
```
[NAV] ========================================
[NAV] Goal RECEIVED: (3.000, 5.000) frame="map" vision=True
[NAV] vision=True → will stop ~1.00 m short of (3.000, 5.000); on-board camera handles the rest.
[NAV] ========================================
[NAV] Published status: "navigating"
[NAV] Simulating navigation to (3.000, 5.000) but stopping ~1.00 m short (vision=True) — waiting 5.0s...
[NAV] Navigation COMPLETE (vision standoff) -> "point_reached" near (3.000, 5.000) (stopped 1.00 m short)
```

**Verify:**
```bash
# Should show "navigating" first, then "point_reached":
ros2 topic echo /nav/status
# data: 'navigating'
# data: 'point_reached'      ← terminal status. NOT 'success'.
```

> ⚠️ The terminal nav status is **`point_reached`** (or `failure`),
> not `success` as the master uses for other stacks. The master maps
> `point_reached → success` internally when it forwards feedback to
> OmniLink.

### Test 2: Arm Stack (runs on Jetson)

```bash
# 2a. Send absolute arm goal (frame=base_link → real cartesian move)
ros2 topic pub --once /supervisor/arm_nav_goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "base_link"}, pose: {position: {x: 0.3, y: 0.2, z: 0.15}, orientation: {w: 1.0}}}'
```

**Watch Terminal 2 (Jetson):**
```
[ARM] ========================================
[ARM] Arm goal RECEIVED: (0.300, 0.200, 0.150) frame="base_link"
[ARM] ========================================
[ARM] Moving to (0.300, 0.200, 0.150) — waiting 2.0s...
[ARM] Move COMPLETE -> "success" pos=(0.300, 0.200, 0.150)
```

**Verify:**
```bash
ros2 topic echo /arm/arm_status --once
# data: 'success'
ros2 topic echo /arm/current_pose --once
# Should reflect the new pose (TRANSIENT_LOCAL — latched for vision node)
```

```bash
# 2b. Close the gripper
ros2 topic pub --once /supervisor/grip std_msgs/Bool '{data: true}'
```

**Watch Terminal 2 (Jetson):**
```
[ARM] ========================================
[ARM] Grip command RECEIVED: CLOSE (grip)
[ARM] ========================================
[ARM] Gripper close — waiting 1.0s...
[ARM] Gripper CLOSED -> "successful_grip"
[ARM] Touch sensor -> true
```

**Verify:**
```bash
ros2 topic echo /arm/gripper_status --once   # data: 'successful_grip'
ros2 topic echo /arm/touch_status   --once   # data: true
```

> Frames matter: `frame_id="base_link"` (or empty) is an absolute move;
> `frame_id="calibration"` is the small relative step the vision node
> uses for camera-2 calibration. Both publish `"success"` to
> `/arm/arm_status` on completion.

### Test 3: Spectrometer Stack (runs on Jetson)

Simulation has been removed — the node **must** be able to reach a
real spectrometer on TCP ports **5000 (read) / 5001 (write)** at the
configured host. If the sockets fail to open, you'll see
`[HW] Could not reach spectrometer ...` and the node will keep
publishing `failure` on every target. To change the host, edit
`src/anubix_spectrometer/config/spectrometer_params.yaml` (params
`host`, `read_port`, `write_port`) and rebuild.

```bash
# 3a. Legacy form (no IDs — Supabase will warn and fall back to
#     yaml-configured defaults):
ros2 topic pub --once /supervisor/spectral_target std_msgs/String \
  '{data: "disease"}'

# 3b. Preferred form (UUIDs forwarded into the Supabase row so the
#     reading is attributed to the right robot/task):
ros2 topic pub --once /supervisor/spectral_target std_msgs/String \
  '{data: "disease|34a957fd-d45c-4dbf-8e02-be8e1b5e349a|40e4060b-5bc8-4044-9d71-046fee27a757"}'
```

**Watch Terminal 2 (Jetson):**
```
[SPECTRO] ========================================
[SPECTRO] Target RECEIVED: task="disease" robot_id="34a957fd-..." task_id="40e4060b-..."
[SPECTRO] ========================================
[SPECTRO] Starting pipeline for task: "disease"
[SPECTRO] Status published: "reading"
[SPECTRO] Status published: "applying_ML"
[SPECTRO] Analysis complete: classification="healthy" confidence=92.00% value=0.0000
[SPECTRO] Details: {'red_edge_ratio': 1.45, 'nir_red_ratio': 2.31, 'chlorophyll_index': 0.18, 'disease_score': 0.0}
[SPECTRO] Status published: "uploading"
[SPECTRO] Status published: "success"
[SPECTRO] Result published to /spectrometer/result
```

**Verify:**
```bash
ros2 topic echo /spectrometer/status --once
# data: 'success'

ros2 topic echo /spectrometer/result --once
# data: '{"task_type":"disease","value":0.0,"classification":"healthy",
#         "confidence":0.92,"details":{...},"timestamp":...,
#         "robot_id":"34a957fd-...","task_id":"40e4060b-..."}'
```

> Valid task values: `water_stress | disease | harvest_status`. Anything
> else falls through to `classification="unknown"` and confidence=0.

### Test 4: Supabase Uploader (runs on Jetson, triggered by Test 3)

The uploader subscribes to `/spectrometer/result`, so every successful
Test 3 run auto-fires an upload. No separate publish is needed.

**Watch Terminal 2 (Jetson) right after Test 3:**
```
[SUPABASE] /spectrometer/result received [total=1] — dispatching upload
[SUPABASE] Payload — task_type='disease' classification='healthy' value=0.0 confidence=0.92
[SUPABASE] Using IDs from spectrometer payload: robot_id='34a957fd-...' task_id='40e4060b-...'
[SUPABASE] Step 1/2 — capturing plant photo from USB camera
[SUPABASE] Opening USB camera index=0
[SUPABASE] Photo saved locally: /tmp/anubix_scan_...jpg
[SUPABASE] Photo uploaded → https://bdkutmmrcjckaazzzspe.supabase.co/storage/v1/object/public/plant-images/scan_...jpg
[SUPABASE] Step 2/2 — building ReadingModel: classification='healthy' ...
[SUPABASE] DB insert attempt 1/3
[SUPABASE] Upload SUCCESS on attempt 1  (row_id=...)
```

**Verify:**
```bash
ros2 topic echo /supabase/upload_status --once
# data: 'success'
```

> If the camera is missing/in-use, the uploader logs a warning, sets
> `photo_url=null`, and the DB insert still proceeds. If the
> spectrometer payload had no `robot_id`/`task_id`, you'll see a
> `[SUPABASE] Spectrometer payload missing IDs ... falling back to
> node params` warning instead of the "Using IDs from payload" line.

### Test 5: Vision Stack (runs on Jetson)

Two cameras, two very different code paths. Pick the camera **before**
sending the perception goal (the camera selection is latched).

#### Camera 1 (RealSense, base) — depth-based 3D, no gripper

```bash
ros2 topic pub --once /supervisor/target_camera std_msgs/String '{data: "1"}'
ros2 topic pub --once /supervisor/perception_goal std_msgs/String '{data: "disease"}'
```

**Watch Terminal 2 (Jetson):**
```
[VISION] Camera set -> 1
[VISION] ========================================
[VISION] perception_goal RECEIVED: task="disease" camera=1
[VISION] ========================================
[VISION] Starting RealSense pipeline...
[VISION] TARGET LEAF FOUND! pixel=(320,240) 3D=(0.4500, 0.1200, 0.8500) m depth=0.850 m
[VISION] Published /perception/target_pose and /perception/status="found"
[VISION] RealSense pipeline stopped
```

**Verify:**
```bash
ros2 topic echo /perception/status      --once   # data: 'found'
ros2 topic echo /perception/target_pose --once   # 3D position in metres
```

> Camera 1 no longer has gripper-pixel parameters or a crosshair
> overlay — those only made sense on the flange-mounted camera 2.

#### Camera 2 (USB flange-mounted) — closest-to-gripper + 2-phase calibration

Camera 2 picks the leaf closest to the gripper pixel, then commands
the arm to move 1 cm right, re-identifies the **same** leaf in a
post-move frame, and uses the pixel displacement to derive a
pixels-per-cm calibration. The arm node must be running for this
test to complete.

```bash
ros2 topic pub --once /supervisor/target_camera std_msgs/String '{data: "2"}'
ros2 topic pub --once /supervisor/perception_goal std_msgs/String '{data: "disease"}'
```

**Watch Terminal 2 (Jetson):**
```
[VISION] Camera set -> 2
[VISION] perception_goal RECEIVED: task="disease" camera=2
[VISION] Opening USB camera at index 0...
[VISION] USB camera opened: 640x480 px, gripper pixel=(320,240)
[VISION] === USB Phase 1: closest-leaf-to-gripper ===
[VISION] Phase 1 COMPLETE — closest leaf at (320, 240)
[VISION] Waiting for /arm/arm_status="success" (timeout=30s)...
[ARM]    Arm goal RECEIVED: (...) frame="calibration"           ← arm receives calibration step
[ARM]    Move COMPLETE -> "success" pos=(...)
[VISION] /arm/arm_status = "success"
[VISION] Arm calibration move CONFIRMED
[VISION] Arm move confirmed — proceeding to Phase 2
[VISION] === USB Phase 2: re-identify same leaf ===
[VISION] Phase 2 COMPLETE — same leaf re-identified at (340, 240)
[VISION] Calibration: 1.00 cm = 20.00 px  ->  1 cm = 20.00 px
[VISION] Offset from gripper: dx=1.00 cm (Right), dy=0.00 cm (Up)
[VISION] Published /perception/target_pose and /perception/status="found"
[VISION] USB camera released
```

**Verify:**
```bash
ros2 topic echo /perception/status      --once   # data: 'found'
ros2 topic echo /perception/target_pose --once   # x/y in metres (cm/100)
```

> Phase 2 uses nearest-centroid matching against the Phase-1 centroid
> with a sanity radius (param `tracking_max_dist_px`, default 200 px)
> so the calibration cannot accidentally lock onto a different leaf
> between frames. If no match within radius: `[VISION] Phase 2 FAILED`
> → `/perception/status = "not_found"`.

---

## Phase 4: End-to-End Mission Test via OmniLink

This tests the full loop: OmniLink AI → Master → All Stacks → Feedback to AI.

### Supervisor command grammar (what the agent prints)

The agent's reply is parsed for these patterns (case-insensitive). Whatever
parameters the OmniLink tool definitions need to accept must produce strings
that match these regexes:

| Command | Example | Meaning |
|---|---|---|
| `supervisor/force_stop` | `supervisor/force_stop` | Abort everything |
| `supervisor/nav_goal_home` | `supervisor/nav_goal_home` | Drive to (home_x, home_y) |
| `supervisor/nav_goal_<x>_<y>` | `supervisor/nav_goal_3_5` | Drive to (3, 5); vision=False |
| `supervisor/nav_goal_<x>_<y>_vision-<bool>` | `supervisor/nav_goal_3_5_vision-true` | Drive to (3, 5) but stop 1 m short so the camera can take over |
| `supervisor/target_camera_<n>` | `supervisor/target_camera_2` | Switch active camera |
| `supervisor/perception_goal_<task>` | `supervisor/perception_goal_disease` | Run vision for this task |
| `supervisor/arm_nav_goal_<signal>` | `supervisor/arm_nav_goal_move` | `move` = go to perception target, `home` = retract |
| `supervisor/grip_<bool>` | `supervisor/grip_true` | Close (`true`) or open (`false`) gripper |
| `supervisor/spectral_target_<task>` | `supervisor/spectral_target_disease` | Run spectrometer (no IDs — uses node defaults) |
| `supervisor/spectral_target_<task>\|<robot_id>\|<task_id>` | `supervisor/spectral_target_disease\|34a957fd-...\|40e4060b-...` | Run spectrometer and tag the Supabase row with these UUIDs |

### OmniLink tool definitions

Two tools accept extra fields that must be reflected in the OmniLink web UI:

- **`navigate`** — add a boolean argument `vision` (default `false`).
  When the tool fires, the agent text must include
  `supervisor/nav_goal_<x>_<y>_vision-<true|false>`. If `vision: true` the
  RPi nav stack stops `vision_standoff_m` (default 1.0 m) short of the goal.

- **`spectrometer`** — add two string arguments `robot_id` and `task_id`
  (UUIDs). When the tool fires, the agent text must include
  `supervisor/spectral_target_<task>|<robot_id>|<task_id>`. Those IDs are
  forwarded all the way to the Supabase row so each reading is attributed
  to the correct robot and task without relying on hardcoded defaults.

### On the OmniLink Web UI

1. Go to https://omnilink.ai (or wherever the agent is hosted)
2. Find agent "ANUBIX" (or create one with the profile from your original files)
3. Send this mission in the chat:

```
Go to plant at coordinates 3,5. Check for disease using camera 1. If found, move the arm to the target and grip it. Then run the spectrometer to confirm disease status.
```

### Watch All Terminals

**Terminal 2 (Jetson) — You should see:**

```
[POLL] Tick #N — checking OmniLink memory...
[POLL] >>> 1 command(s) detected: ['supervisor/nav_goal_3_5_vision-true']
[CMD] >>> supervisor/nav_goal_3_5_vision-true
[TX] /supervisor/nav_goal (3.00, 5.00) vision=True
   ← on RPi terminal: [NAV] Goal RECEIVED: (3.000, 5.000) vision=True
   ← on RPi terminal: [NAV] Navigation COMPLETE (vision standoff) -> "point_reached"
[RX] /nav/status = "point_reached"
[FEEDBACK -> ANUBIX] /nav/status: point_reached

[POLL] >>> 2 command(s) detected: ['supervisor/target_camera_1', 'supervisor/perception_goal_disease']
[CMD] >>> supervisor/target_camera_1
[TX] /supervisor/target_camera = 1
[CMD] >>> supervisor/perception_goal_disease
[TX] /supervisor/perception_goal = "disease"
[VISION] perception_goal RECEIVED: task="disease" camera=1
[VISION] TARGET LEAF FOUND! pixel=(320,240) 3D=(0.4500, 0.1200, 0.8500) m depth=0.850 m
[RX] /perception/status = "found"
[RX] /perception/target_pose = (0.450, 0.120, 0.850)
[FEEDBACK -> ANUBIX] /perception/status: found

[CMD] >>> supervisor/arm_nav_goal_move
[TX] /supervisor/arm_nav_goal (signal=move, dest=target)
[ARM] Arm goal RECEIVED: (0.450, 0.120, 0.850) frame="base_link"
[ARM] Move COMPLETE -> "success" pos=(0.450, 0.120, 0.850)
[RX] /arm/arm_status = "success"

[CMD] >>> supervisor/grip_true
[TX] /supervisor/grip = True (close)
[ARM] Grip command RECEIVED: CLOSE (grip)
[ARM] Gripper CLOSED -> "successful_grip"
[ARM] Touch sensor -> true
[RX] /arm/gripper_status = "successful_grip"
[RX] /arm/touch_status = True
[FEEDBACK -> ANUBIX] /arm/gripper_status: successful_grip | /arm/touch_status: true

[CMD] >>> supervisor/spectral_target_disease|34a957fd-...|40e4060b-...
[TX] /supervisor/spectral_target = "disease|34a957fd-...|40e4060b-..."
[SPECTRO] Target RECEIVED: task="disease" robot_id="34a957fd-..." task_id="40e4060b-..."
[SPECTRO] Status published: "reading"
[SPECTRO] Status published: "applying_ML"
[SPECTRO] Status published: "uploading"
[SPECTRO] Status published: "success"
[SPECTRO] Result published to /spectrometer/result
[RX] /spectrometer/status = "success"
[FEEDBACK -> ANUBIX] /spectrometer/status: success

[SUPABASE] /spectrometer/result received [total=1] — dispatching upload
[SUPABASE] Using IDs from spectrometer payload: robot_id='34a957fd-...' task_id='40e4060b-...'
[SUPABASE] Step 1/2 — capturing plant photo from USB camera
[SUPABASE] Photo uploaded → https://...
[SUPABASE] Step 2/2 — building ReadingModel: ...
[SUPABASE] Upload SUCCESS on attempt 1
```

**Terminal 1 (RPi) — You should see:**

```
[NAV] ========================================
[NAV] Goal RECEIVED: (3.000, 5.000) frame="map" vision=True
[NAV] vision=True → will stop ~1.00 m short of (3.000, 5.000); on-board camera handles the rest.
[NAV] ========================================
[NAV] Published status: "navigating"
[NAV] Navigation COMPLETE (vision standoff) -> "point_reached" near (3.000, 5.000) (stopped 1.00 m short)
[anubix_rpi_bridge]: [DIAGNOSTICS] Jetson heartbeat OK (last seen 0.2s ago)
```

### Check OmniLink Chat

The AI agent should respond with something like:

```
Mission complete. I navigated to plant at (3, 5), detected a leaf using the depth camera, moved the arm to position (0.45m, 0.12m, 0.85m), successfully gripped it (touch sensor confirmed), and ran the spectrometer. The spectral analysis shows the plant is healthy. The reading has been uploaded to Supabase with robot_id 34a957fd-d45c-4dbf-8e02-be8e1b5e349a and task_id 40e4060b-5bc8-4044-9d71-046fee27a757.
```

---

## Phase 5: Verify Supabase Upload

1. Go to your Supabase dashboard: https://supabase.com/dashboard
2. Navigate to **Table Editor** → `readings` table
3. You should see a new row with:
   - `robot_id`: 34a957fd-d45c-4dbf-8e02-be8e1b5e349a
   - `task_id`: 40e4060b-5bc8-4044-9d71-046fee27a757
   - `plant_location`: "0,0" (or whatever you set in `supabase_params.yaml`)
   - `disease_detected`: true/false
   - `disease_name`: "TMV" or "none"
   - `photo_1_url`: https://bdkutmmrcjckaazzzspe.supabase.co/storage/v1/object/public/plant-images/scan_...jpg
   - `recorded_at`: timestamp

4. Click the photo URL — should open the captured plant image in your browser

---

## Phase 6: Emergency Stop Test

```bash
# Terminal 3 — Trigger emergency stop
ros2 topic pub --once /supervisor/force_stop std_msgs/Bool '{data: true}'

# Watch Terminal 1 (RPi) — should see:
# [NAV] *** FORCE STOP RECEIVED *** — ignoring future goals
# [anubix_rpi_bridge]: ALERT: Force stop received from Jetson

# Watch Terminal 2 (Jetson) — should see:
# *** FORCE STOP PUBLISHED ***                      ← from master
# [ARM]     *** FORCE STOP RECEIVED *** — halting all arm operations
# [SPECTRO] *** FORCE STOP RECEIVED ***
# [VISION]  *** FORCE STOP *** — aborting pipeline

# Try sending a command — should be rejected:
ros2 topic pub --once /supervisor/nav_goal geometry_msgs/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}'

# Terminal 1 (RPi) — should see:
# [NAV] REJECTED: robot is force_stopped. Publishing "failure".
ros2 topic echo /nav/status --once   # data: 'failure'

# To recover: restart both launch files (Ctrl+C, then relaunch)
```

---

## Troubleshooting Commands

### If topics don't appear across machines

```bash
# On both machines:
ros2 daemon stop && ros2 daemon start

# Check DDS discovery:
ros2 topic list -v
ros2 node list

# Check if the peer address is correct:
cat ~/.ros/jetson_cyclone.xml | grep Peer
cat ~/.ros/rpi_cyclone.xml | grep Peer

# Manually test DDS with a simple publisher/subscriber:
# Terminal on Jetson:
ros2 topic pub /test std_msgs/String '{data: "hello from jetson"}'

# Terminal on RPi:
ros2 topic echo /test
# Should see: data: 'hello from jetson'
```

### If OmniLink doesn't respond

```bash
# Check if the master is actually polling:
# Terminal 2 (Jetson) — look for periodic log lines like:
# [anubix_master]: [POLL] ...

# Check the OmniLink agent name matches:
ros2 param get /anubix_master agent_name
# Should return: ANUBIX

# Test OmniLink key directly (outside ROS):
python3 -c "
from omnilink.client import OmniLinkClient
client = OmniLinkClient(omni_key='olink_4ekYIgHACfZaGlq6WJOgu59U')
memory = client.get_memory('ANUBIX')
print(f'Memory length: {len(memory)} messages')
"
```

### If camera doesn't work

```bash
# Check if camera device exists:
ls -l /dev/video*

# Test camera capture manually:
python3 << 'EOF'
import cv2
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cap.isOpened():
    print("ERROR: Cannot open camera")
else:
    ret, frame = cap.read()
    print(f"Capture OK: {ret}, frame shape: {frame.shape if ret else 'N/A'}")
    cap.release()
EOF

# Check RealSense (if you have it):
rs-enumerate-devices
# Should list connected RealSense cameras
```

### If Supabase upload fails

```bash
# Watch the Supabase node logs closely — it logs every attempt:
ros2 topic echo /rosout | grep SUPABASE

# Test Supabase connection directly:
python3 << 'EOF'
from supabase import create_client
url = "https://bdkutmmrcjckaazzzspe.supabase.co"
key = "sb_publishable_VY6-Jjc6f20Wcbb3Rm8gwg_ZK6CYuh3"
client = create_client(url, key)
result = client.table('readings').select('*').limit(1).execute()
print(f"Supabase connection OK, found {len(result.data)} rows")
EOF
```

### Raspberry Pi OS specific issues

#### Network config doesn't persist after reboot

```bash
# Check if dhcpcd is running (Raspberry Pi OS default)
systemctl status dhcpcd

# Verify dhcpcd config
cat /etc/dhcpcd.conf | grep -A3 "ANUBIX"
# Should show:
# interface eth0
# static ip_address=192.168.10.2/24
# nolink

# If missing, manually add:
sudo nano /etc/dhcpcd.conf
# Add at the end:
interface eth0
static ip_address=192.168.10.2/24
nolink

# Restart dhcpcd
sudo systemctl restart dhcpcd
```

#### ROS 2 packages missing after install

Raspberry Pi OS (Debian) isn't officially supported by ROS 2, but it works using the Ubuntu Jammy repository. If you get dependency errors:

```bash
sudo apt-get update
sudo apt-get install -f
sudo apt-get install --fix-missing
```

#### pip install fails with "externally-managed-environment"

Debian 12+ (Bookworm/Trixie) requires `--break-system-packages` flag:

```bash
pip3 install --break-system-packages <package>
```

The install script handles this automatically.

---

## Summary of Success Indicators

✅ **Network**: Both machines can ping each other  
✅ **DDS**: Topics from both sides visible with `ros2 topic list`  
✅ **Heartbeats**: `/bridge/jetson_heartbeat` and `/bridge/rpi_heartbeat` flowing at 1 Hz  
✅ **Navigation**: Publishes `navigating` → `point_reached` on `/nav/status`; respects `/supervisor/nav_vision` standoff  
✅ **Arm**: Publishes `success` on `/arm/arm_status`, `successful_grip` on `/arm/gripper_status`, `true` on `/arm/touch_status`  
✅ **Spectrometer**: Connects to TCP `host:5000`/`5001`, publishes `success` on `/spectrometer/status`, JSON on `/spectrometer/result` (no simulate mode)  
✅ **Vision**: Publishes `found` on `/perception/status`, 3D Pose on `/perception/target_pose`; camera 2 performs 2-phase calibration via the arm  
✅ **Supabase**: Publishes `success` on `/supabase/upload_status`, new row appears in dashboard with payload-supplied robot/task IDs  
✅ **OmniLink**: Master sends feedback, AI responds with next command, full mission completes  

**If all these pass → ANUBIX is fully operational 🚀**
