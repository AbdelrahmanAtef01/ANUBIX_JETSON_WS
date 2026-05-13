#!/usr/bin/env python3
"""
Pure-function leaf detection helpers extracted from the original vision system.
No ROS dependencies — safe to unit-test standalone.
"""

import cv2
import numpy as np

CLASS_HEALTHY_LEAF = 2


def extract_healthy_leaves(results):
    """Return a list of healthy-leaf dicts from a YOLO segmentation result.

    Each dict: {'centroid': (cx, cy), 'lowest_y': int, 'pts': np.ndarray}
    """
    leaves = []
    if results[0].masks is None:
        return leaves

    for i, mask in enumerate(results[0].masks.xy):
        cls = int(results[0].boxes.cls[i])
        if cls != CLASS_HEALTHY_LEAF:
            continue

        mask_pts = mask.astype(int)
        moments = cv2.moments(mask_pts)
        if moments['m00'] == 0:
            continue

        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        leaves.append({
            'centroid': (cx, cy),
            'lowest_y': int(np.max(mask_pts[:, 1])),
            'pts': mask_pts,
        })

    return leaves


def get_target_leaf(results, frame_w, frame_h):
    """
    Filtration logic to find the optimal target leaf from YOLO segmentation results.

    Prefers the left half of the frame; falls back to right when no left leaves exist.
    Scores candidates by proximity to the bottom-centre grab plane plus a penalty for
    leaves that are not near the vertical middle of the plant.

    Returns:
        (all_healthy_leaves, best_target_leaf)
    """
    healthy_leaves = extract_healthy_leaves(results)
    if not healthy_leaves:
        return [], None

    has_left = any(l['centroid'][0] < frame_w // 2 for l in healthy_leaves)
    has_right = any(l['centroid'][0] >= frame_w // 2 for l in healthy_leaves)

    if has_left:
        selected = [l for l in healthy_leaves if l['centroid'][0] < frame_w // 2]
    elif has_right:
        selected = [l for l in healthy_leaves if l['centroid'][0] >= frame_w // 2]
    else:
        return healthy_leaves, None

    all_y = [l['centroid'][1] for l in selected]
    min_y, max_y = min(all_y), max(all_y)
    plant_height = max_y - min_y

    roi_center_x = frame_w // 2
    roi_bottom_y = frame_h - 20

    best_score = float('inf')
    target_leaf = None

    for leaf in selected:
        dist_to_plane = np.sqrt(
            (leaf['centroid'][0] - roi_center_x) ** 2 +
            (leaf['lowest_y'] - roi_bottom_y) ** 2
        )
        relative_y = (leaf['centroid'][1] - min_y) / (plant_height if plant_height > 0 else 1)
        middle_penalty = 0.0 if 0.4 <= relative_y <= 0.6 else abs(relative_y - 0.5) * 500.0
        score = dist_to_plane + middle_penalty

        if score < best_score:
            best_score = score
            target_leaf = leaf

    return healthy_leaves, target_leaf


def get_closest_leaf_to_gripper(results, gripper_x, gripper_y):
    """Camera-2 (flange) selection: pick the healthy leaf whose centroid is
    nearest to the gripper pixel position. Returns (all_leaves, target_leaf)."""
    healthy_leaves = extract_healthy_leaves(results)
    if not healthy_leaves:
        return [], None

    best = None
    best_dist = float('inf')
    for leaf in healthy_leaves:
        cx, cy = leaf['centroid']
        d = (cx - gripper_x) ** 2 + (cy - gripper_y) ** 2
        if d < best_dist:
            best_dist = d
            best = leaf

    return healthy_leaves, best


def match_closest_leaf(results, anchor_centroid, max_dist_px=200):
    """Re-identify the same leaf in a later frame by picking the detection
    whose centroid is nearest to ``anchor_centroid``. Returns
    (all_leaves, matched_leaf, distance_px). matched_leaf is None if the
    nearest candidate is farther than ``max_dist_px``."""
    healthy_leaves = extract_healthy_leaves(results)
    if not healthy_leaves:
        return [], None, float('inf')

    ax, ay = anchor_centroid
    best = None
    best_dist = float('inf')
    for leaf in healthy_leaves:
        cx, cy = leaf['centroid']
        d = float(np.sqrt((cx - ax) ** 2 + (cy - ay) ** 2))
        if d < best_dist:
            best_dist = d
            best = leaf

    if best is None or best_dist > max_dist_px:
        return healthy_leaves, None, best_dist

    return healthy_leaves, best, best_dist


def draw_leaves(frame, all_leaves, target_leaf):
    """Draw all healthy leaves in blue; target leaf outline in green with centroid dot."""
    for leaf in all_leaves:
        cv2.polylines(frame, [leaf['pts']], True, (255, 0, 0), 2)

    if target_leaf:
        cv2.polylines(frame, [target_leaf['pts']], True, (0, 255, 0), 4)
        cx, cy = target_leaf['centroid']
        cv2.circle(frame, (cx, cy), 7, (0, 0, 255), -1)
        cv2.putText(frame, 'TARGET LEAF', (cx - 50, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)


def draw_grabber_ui(frame, gx, gy):
    """Draw crosshair at the grabber centre position."""
    cv2.line(frame, (gx - 15, gy), (gx + 15, gy), (255, 0, 0), 2)
    cv2.line(frame, (gx, gy - 15), (gx, gy + 15), (255, 0, 0), 2)
    cv2.putText(frame, f'GRABBER (X:{gx}, Y:{gy})', (gx - 75, gy - 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)


def draw_hud(frame, lines, origin=(10, 25)):
    """Render a stack of status lines on the top-left."""
    x, y = origin
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (x, y + 22 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
