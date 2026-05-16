#!/usr/bin/env python3
"""
Command parsing for ANUBIX supervisor commands.
Extracts structured commands from ANUBIX agent text responses.
"""

import re
from typing import List, Tuple

# UUID pattern fragment (relaxed: allows any hex+dash combo of plausible length)
_UUID_FRAG = r'[A-Za-z0-9\-]{8,}'

CMD_PATTERNS = [
    ("force_stop", re.compile(r'supervisor/force_stop', re.IGNORECASE)),
    # Standalone robot_id and task_id commands (set context for subsequent commands)
    ("robot_id", re.compile(r'supervisor/robot_id_(' + _UUID_FRAG + r')', re.IGNORECASE)),
    ("task_id", re.compile(r'supervisor/task_id_(' + _UUID_FRAG + r')', re.IGNORECASE)),
    ("nav_goal_home", re.compile(r'supervisor/nav_goal_home(?:_home)?', re.IGNORECASE)),
    # nav_goal: x_y or x_y|robot_id|task_id (pipe-separated like spectrometer)
    # Examples: nav_goal_3_5, nav_goal_3_5|robot-uuid|task-uuid
    # IDs are published separately to /supervisor/robot_id and /supervisor/task_id
    # to keep navigation geometry clean
    ("nav_goal", re.compile(
        r'supervisor/nav_goal_([-\d]+(?:\.\d+)?)_([-\d]+(?:\.\d+)?)'
        r'(?:\|(' + _UUID_FRAG + r'))?'
        r'(?:\|(' + _UUID_FRAG + r'))?',
        re.IGNORECASE)),
    # nav_vision: separate command for vision mode
    ("nav_vision", re.compile(
        r'supervisor/nav_vision_(true|false)',
        re.IGNORECASE)),
    ("perception_goal", re.compile(
        r'supervisor/perception_goal_([a-z][a-z_]*)',
        re.IGNORECASE)),
    ("target_camera", re.compile(
        r'supervisor/target_camera_(\d+)',
        re.IGNORECASE)),
    ("arm_nav_goal", re.compile(
        r'supervisor/arm_nav_goal_([a-z]+)',
        re.IGNORECASE)),
    ("grip", re.compile(
        r'supervisor/grip_(true|false)',
        re.IGNORECASE)),
    # spectral_target: task                                (legacy)
    #                  task|robot_id|task_id              (preferred)
    ("spectral_target", re.compile(
        r'supervisor/spectral_target_([a-z][a-z_]*)'
        r'(?:\|(' + _UUID_FRAG + r'))?'
        r'(?:\|(' + _UUID_FRAG + r'))?',
        re.IGNORECASE)),
]

CMD_PRIORITY = {
    "force_stop": 0,
    "robot_id": 1,  # Set context first
    "task_id": 1,   # Set context first
    "nav_goal_home": 2,
    "nav_goal": 3,
    "nav_vision": 3,  # Same priority as nav_goal, can be sent together
    "target_camera": 4,
    "perception_goal": 5,
    "arm_nav_goal": 6,
    "grip": 7,
    "spectral_target": 8,
}


def parse_commands(text: str) -> List[Tuple[str, re.Match]]:
    """Parse ANUBIX text output and return list of (cmd_type, match) tuples."""
    results = []
    for cmd_type, pattern in CMD_PATTERNS:
        for match in pattern.finditer(text):
            results.append((cmd_type, match))
    return results
