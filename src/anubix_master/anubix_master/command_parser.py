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
    # CRITICAL EXECUTION ORDER CONSTRAINTS:
    # Lower numbers execute first. DO NOT change these without understanding the dependencies.
    #
    # Priority groups:
    # 0: Emergency stop (always first)
    # 1: Context setup (robot_id, task_id) - MUST execute before commands that need them
    # 2: Mode configuration (nav_vision) - MUST execute before the action it configures
    # 3-4: Navigation commands
    # 5-6: Perception commands (target_camera BEFORE perception_goal)
    # 7-9: Manipulation and sensing commands
    #
    # NEVER assign the same priority to commands with ordering dependencies!
    # See validate_priorities() below for automatic validation.

    "force_stop": 0,         # Emergency abort - always first
    "robot_id": 1,           # Set robot context - needed by nav_goal and spectral_target
    "task_id": 1,            # Set task context - needed by nav_goal and spectral_target
    "nav_vision": 2,         # CRITICAL: Set vision mode BEFORE nav_goal (stops 1m early if true)
    "nav_goal_home": 3,      # Return to home position
    "nav_goal": 4,           # Navigate to coordinates (MUST execute AFTER nav_vision)
    "target_camera": 5,      # Select camera BEFORE starting perception
    "perception_goal": 6,    # Start perception (uses camera set by target_camera)
    "arm_nav_goal": 7,       # Move arm
    "grip": 8,               # Gripper control
    "spectral_target": 9,    # Spectrometer scan (uses robot_id/task_id from context)
}


def parse_commands(text: str) -> List[Tuple[str, re.Match]]:
    """Parse ANUBIX text output and return list of (cmd_type, match) tuples."""
    results = []
    for cmd_type, pattern in CMD_PATTERNS:
        for match in pattern.finditer(text):
            results.append((cmd_type, match))
    return results


def validate_priorities() -> None:
    """
    Validate command priorities to ensure critical execution order constraints:
    - nav_vision MUST execute before nav_goal (vision mode must be set first)
    - target_camera SHOULD execute before perception_goal (camera must be selected)
    - robot_id/task_id MUST execute before commands that need them

    Raises ValueError if constraints are violated.
    """
    issues = []

    # Critical: nav_vision must have lower priority number than nav_goal
    if CMD_PRIORITY.get('nav_vision', 99) >= CMD_PRIORITY.get('nav_goal', 99):
        issues.append(
            'nav_vision must have LOWER priority number than nav_goal '
            '(vision mode must be set before navigation starts)')

    # Important: target_camera should execute before perception_goal
    if CMD_PRIORITY.get('target_camera', 99) >= CMD_PRIORITY.get('perception_goal', 99):
        issues.append(
            'target_camera should have LOWER priority number than perception_goal '
            '(camera must be selected before perception starts)')

    # Context IDs should execute first
    id_priority = max(CMD_PRIORITY.get('robot_id', 0), CMD_PRIORITY.get('task_id', 0))
    for cmd in ['nav_goal', 'spectral_target']:
        if id_priority >= CMD_PRIORITY.get(cmd, 99):
            issues.append(
                f'robot_id/task_id must have LOWER priority number than {cmd} '
                '(IDs must be set before commands that need them)')

    if issues:
        raise ValueError(
            'Command priority validation failed:\n  - ' + '\n  - '.join(issues))


# Validate at module load time
validate_priorities()
