#!/usr/bin/env python3
"""
Pure-function leaf detection helpers extracted from the original vision system.
No ROS dependencies — safe to unit-test standalone.
"""

import cv2
import numpy as np

CLASS_HEALTHY_LEAF = 2


def get_target_leaf(results, frame_w, frame_h):
    """
    Filtration logic to find the optimal target leaf from YOLO segmentation results.

    Prefers the left half of the frame; falls back to right when no left leaves exist.
    Scores candidates by proximity to the bottom-centre grab plane plus a penalty for
    leaves that are not near the vertical middle of the plant.

    Returns:
        (all_healthy_leaves, best_target_leaf)
        Each leaf dict: {'centroid': (cx, cy), 'lowest_y': int, 'pts': np.ndarray}
    """
    healthy_leaves = []
    has_left = False
    has_right = False

    if results[0].masks is None:
        return [], None

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

        if cx < frame_w // 2:
            has_left = True
        else:
            has_right = True

        healthy_leaves.append({
            'centroid': (cx, cy),
            'lowest_y': int(np.max(mask_pts[:, 1])),
            'pts': mask_pts,
        })

    if not healthy_leaves:
        return [], None

    # Prefer left side; only use right if no left leaves exist
    if has_left:
        selected = [l for l in healthy_leaves if l['centroid'][0] < frame_w // 2]
    else:
        selected = [l for l in healthy_leaves if l['centroid'][0] >= frame_w // 2]

    if not selected:
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
