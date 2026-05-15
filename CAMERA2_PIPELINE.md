# Camera 2 (USB Flange-Mounted) Pipeline - Detailed Documentation

## Overview

Camera 2 is a **monocular USB camera** mounted on the arm's flange (end-effector). Unlike Camera 1 (RealSense) which provides 3D depth directly, Camera 2 uses a **calibration-based distance measurement** technique.

## Key Characteristics

- **Camera Type**: USB mono camera (no depth sensor)
- **Mounting**: Fixed to arm flange/gripper assembly
- **Field of View**: Sees the workspace from the gripper's perspective
- **Depth Capability**: ❌ **NO** - Camera 2 does NOT calculate Z depth
- **Output**: 2D offset (X, Y) in centimeters relative to gripper position

## Full Pipeline Flow

### Phase 1: Initial Leaf Detection

**Goal**: Find the leaf closest to the gripper pixel position

1. **Gripper Position Resolution**
   - Parameter: `gripper_px_x_cam2`, `gripper_px_y_cam2` (config)
   - If set to -1, defaults to frame center
   - This represents where the gripper appears in the camera's view

2. **Detection Loop** (up to `detection_timeout_s`, default 30s)
   - Capture frame from USB camera
   - Run YOLO segmentation: `model.predict(frame, conf=confidence)`
   - Extract all detected leaf instances with centroids
   - Use `get_closest_leaf_to_gripper(results, gx, gy)` to find the nearest leaf
   - **Selection Criterion**: Euclidean distance from gripper pixel to leaf centroid
   - Loop at `detection_rate_hz` (default 2 Hz) until a leaf is found

3. **Phase 1 Output**
   - `centroid_1 = (cx1, cy1)` - pixel coordinates of the selected leaf
   - `leaf_1` - full leaf dict with mask, bbox, centroid

### Phase 2: Calibration via Arm Movement

**Goal**: Determine the pixels-per-cm scale by tracking the SAME leaf after arm movement

4. **Send Calibration Arm Goal**
   - Command arm to move exactly `calibration_step_m` (default 0.01 m = 1 cm) to the RIGHT (+X)
   - Movement is **absolute**: reads current arm pose from `/arm/current_pose`
   - New pose = `(current_x + 0.01, current_y, current_z, same_orientation)`
   - Publishes to `/supervisor/arm_nav_goal` with `frame_id='base_link'`
   - Sets `_waiting_for_arm = True` to filter arm status callbacks

5. **Wait for Arm Confirmation**
   - Listens to `/arm/arm_status` with timeout `arm_move_timeout_s` (default 30s)
   - Only processes "success" when `_waiting_for_arm == True`
   - Ignores arm status from other sources (Camera 1, master commands, etc.)
   - After confirmation: flush 5 frames to clear stale buffered images

6. **Re-identify the SAME Leaf** (Phase 2 Detection Loop)
   - Capture new frame after arm has moved
   - Run YOLO segmentation again
   - Use `match_closest_leaf(results, anchor=centroid_1, max_dist_px=tracking_max_dist)`
   - **Matching Logic**: Find leaf with centroid closest to `centroid_1`
   - **Sanity Check**: Distance must be < `tracking_max_dist_px` (default 200 px)
   - This prevents locking onto a DIFFERENT leaf
   - Loop at `detection_rate_hz` until match found or timeout

7. **Phase 2 Output**
   - `centroid_2 = (cx2, cy2)` - pixel coordinates of the SAME leaf after arm move

### Phase 3: Calibration & Distance Calculation

8. **Calculate Pixels-Per-Centimeter Scale**
   ```python
   dist_px = sqrt((cx2 - cx1)² + (cy2 - cy1)²)  # Pixel displacement of leaf
   calibration_cm = calibration_step_m * 100   # 1.0 cm
   pixels_per_cm = dist_px / calibration_cm
   ```
   - **Example**: If leaf moved 20 pixels when arm moved 1 cm → 1 cm = 20 px

9. **Calculate Leaf Offset from Gripper**
   ```python
   dx_px = cx2 - gx  # Horizontal pixel offset
   dy_px = cy2 - gy  # Vertical pixel offset
   
   dx_cm = dx_px / pixels_per_cm  # Convert to centimeters
   dy_cm = dy_px / pixels_per_cm
   ```
   - `dx_cm > 0`: Leaf is to the RIGHT of gripper
   - `dx_cm < 0`: Leaf is to the LEFT of gripper
   - `dy_cm > 0`: Leaf is BELOW gripper (image Y increases downward)
   - `dy_cm < 0`: Leaf is ABOVE gripper

10. **Publish Target Pose**
    ```python
    pose.position.x = dx_cm * 0.01   # Convert cm to meters
    pose.position.y = -dy_cm * 0.01  # Negate Y (ROS convention: Y+ is left)
    pose.position.z = 0.0            # ❌ NO depth - set to zero
    ```
    - Published to `/perception/target_pose`
    - Pose is **relative to gripper position** (not absolute)
    - Master node uses this offset to command arm movement

## Z Depth: Why Camera 2 Doesn't Calculate It

**Camera 2 does NOT calculate Z (depth) because:**

1. **Monocular Camera**: No stereo/depth sensor like RealSense
2. **Single-Axis Calibration**: Only moves arm in X direction, can only measure XY plane scale
3. **Gripper Frame**: Operates in the gripper's 2D plane, assumes target is at same Z
4. **Sufficient for Task**: Leaf is approximately at gripper height after Camera 1 positions arm

**If Z were needed:**
- Would require stereo camera pair OR depth sensor
- Or multiple calibration moves in different axes
- Or size-based heuristic (leaf size in pixels → distance)
- Current architecture assumes Camera 1 (RealSense) already positioned arm at correct Z

## Key Advantages of This Approach

✅ **No Depth Sensor Required**: Works with cheap USB webcam  
✅ **Self-Calibrating**: Automatically determines pixel scale  
✅ **Robust to Camera Changes**: Scale computed at runtime, not hardcoded  
✅ **Tracks Same Leaf**: Centroid matching prevents calibration errors from leaf switching  
✅ **Accounts for Lens Distortion**: Measurement is local to leaf position  

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `calibration_step_m` | 0.01 | Arm movement distance for calibration (meters) |
| `gripper_px_x_cam2` | -1 | Gripper X pixel (or -1 for center) |
| `gripper_px_y_cam2` | -1 | Gripper Y pixel (or -1 for center) |
| `tracking_max_dist_px` | 200 | Max pixel distance for leaf re-identification |
| `detection_timeout_s` | 30.0 | Max time for each detection phase |
| `detection_rate_hz` | 2.0 | Detection attempts per second |
| `arm_move_timeout_s` | 30.0 | Max time to wait for arm confirmation |

## Error Handling

- **Phase 1 Fails**: No leaf detected → publish "not_found"
- **Arm Move Timeout**: Calibration failed → publish "not_found"  
- **Phase 2 Fails**: Can't re-identify leaf → publish "not_found"
- **Leaf Didn't Move**: `dist_px < 1.0` → invalid calibration → publish "not_found"
- **Force Stop**: Any phase → abort immediately, publish "not_found"

## Summary

Camera 2 is a **2D calibrated monocular system** that:
1. Identifies a leaf closest to the gripper
2. Tracks it through a known arm movement
3. Computes pixel-to-cm scale from the displacement
4. Reports 2D offset (X, Y) in centimeters
5. **Does NOT** calculate depth (Z=0 always)

This makes it ideal for final approach/grasping after Camera 1 has already positioned the arm near the target.
