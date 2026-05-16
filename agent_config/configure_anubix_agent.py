#!/usr/bin/env python3
"""
ANUBIX Agent Configuration Script for OmniLink v1.0.0

This script configures the ANUBIX agent profile using the OmniLink Python Library.
It reads the v2.0 agent prompt and creates/updates the agent profile programmatically.

Requirements:
- omnilink library (pip install omnilink)
- OMNI_KEY environment variable set with your OmniLink API key
- ANUBIX_AGENT_PROMPT_v2.txt in anubix_ws directory

Usage:
    export OMNI_KEY="olink_your_key_here"
    python3 configure_anubix_agent.py
"""

import os
import sys
from pathlib import Path

try:
    from omnilink.client import OmniLinkClient
except ImportError:
    print("❌ Error: omnilink library not found")
    print("Install with: pip install omnilink")
    sys.exit(1)


def load_agent_prompt():
    """Load the ANUBIX agent prompt from file."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    prompt_path = os.path.join(script_dir, "ANUBIX_AGENT_PROMPT_v3_TOOLCALLS.txt")

    with open(prompt_path, 'r', encoding='utf-8') as f:
        content = f.read()

    print(f"✅ Loaded agent prompt from {prompt_path} ({len(content)} bytes)")
    return content


def configure_agent(api_key, agent_prompt):
    """Create or update the ANUBIX agent profile."""
    try:
        client = OmniLinkClient(omni_key=api_key)
        print("✅ Connected to OmniLink platform")
    except Exception as e:
        print(f"❌ Error: Failed to connect to OmniLink: {e}")
        sys.exit(1)

    # Check if ANUBIX profile already exists
    try:
        profiles = client.list_profiles()
        existing_profile = next((p for p in profiles if p['name'] == 'ANUBIX'), None)
    except Exception as e:
        print(f"❌ Error: Failed to list profiles: {e}")
        sys.exit(1)

    # Define all 11 supervisor tools
    supervisor_tools = [
        {
            "name": "supervisor_robot_id",
            "description": "Set the robot context ID for tracking",
            "parameters": {
                "type": "object",
                "properties": {
                    "robot_id": {
                        "type": "string",
                        "description": "UUID or identifier for the robot instance"
                    }
                },
                "required": ["robot_id"]
            }
        },
        {
            "name": "supervisor_task_id",
            "description": "Set the task context ID for tracking",
            "parameters": {
                "type": "object",
                "properties": {
                    "task_id": {
                        "type": "string",
                        "description": "UUID or identifier for the specific task"
                    }
                },
                "required": ["task_id"]
            }
        },
        {
            "name": "supervisor_nav_vision",
            "description": "Set navigation vision mode (true=stop 1m before target for camera, false=drive all the way)",
            "parameters": {
                "type": "object",
                "properties": {
                    "vision": {
                        "type": "boolean",
                        "description": "Enable vision mode (stop before target)"
                    }
                },
                "required": ["vision"]
            }
        },
        {
            "name": "supervisor_nav_goal",
            "description": "Navigate robot to specific coordinates",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {
                        "type": "number",
                        "description": "X coordinate in meters"
                    },
                    "y": {
                        "type": "number",
                        "description": "Y coordinate in meters"
                    }
                },
                "required": ["x", "y"]
            }
        },
        {
            "name": "supervisor_nav_goal_home",
            "description": "Return robot to home position (0, 0)",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        },
        {
            "name": "supervisor_target_camera",
            "description": "Select camera for perception (1=wide-angle, 2=telephoto)",
            "parameters": {
                "type": "object",
                "properties": {
                    "camera_number": {
                        "type": "integer",
                        "description": "Camera number (1 or 2)",
                        "enum": [1, 2]
                    }
                },
                "required": ["camera_number"]
            }
        },
        {
            "name": "supervisor_perception_goal",
            "description": "Start perception task",
            "parameters": {
                "type": "object",
                "properties": {
                    "task_type": {
                        "type": "string",
                        "description": "Type of perception task",
                        "enum": ["disease", "water_stress", "harvest_status"]
                    }
                },
                "required": ["task_type"]
            }
        },
        {
            "name": "supervisor_arm_nav_goal",
            "description": "Move robotic arm to target or home position",
            "parameters": {
                "type": "object",
                "properties": {
                    "signal": {
                        "type": "string",
                        "description": "move=go to target pose, home=return to safe position",
                        "enum": ["move", "home"]
                    }
                },
                "required": ["signal"]
            }
        },
        {
            "name": "supervisor_grip",
            "description": "Control gripper (open/close)",
            "parameters": {
                "type": "object",
                "properties": {
                    "action": {
                        "type": "boolean",
                        "description": "true=close gripper, false=open gripper"
                    }
                },
                "required": ["action"]
            }
        },
        {
            "name": "supervisor_spectral_target",
            "description": "Run spectrometer scan on target",
            "parameters": {
                "type": "object",
                "properties": {
                    "task_type": {
                        "type": "string",
                        "description": "Type of spectral analysis",
                        "enum": ["disease", "water_stress", "harvest_status"]
                    },
                    "robot_id": {
                        "type": "string",
                        "description": "Robot ID (optional, uses context if not provided)"
                    },
                    "task_id": {
                        "type": "string",
                        "description": "Task ID (optional, uses context if not provided)"
                    }
                },
                "required": ["task_type"]
            }
        },
        {
            "name": "supervisor_force_stop",
            "description": "Emergency stop - halts all robot operations immediately",
            "parameters": {
                "type": "object",
                "properties": {}
            }
        }
    ]

    # Agent settings with tool use enabled
    # toolCallbackUrl: The master node runs an HTTP server on port 5055.
    # The OmniLink web UI frontend will POST tool calls to this URL.
    # If running on a remote machine, change to the machine's IP/hostname.
    tool_callback_url = os.environ.get("ANUBIX_CALLBACK_URL", "http://127.0.0.1:5055/tool")

    agent_settings = {
        "agentName": "ANUBIX",
        "mainTask": agent_prompt,
        "agentPersonality": "Professional",
        "allowToolUse": True,
        "availableTools": ",".join([t["name"] for t in supervisor_tools]),
        "availableToolDetails": supervisor_tools,
        "toolCallbackUrl": tool_callback_url,
        "maxToolRounds": 15,
    }

    # Create or update profile
    try:
        if existing_profile:
            profile_id = existing_profile['id']
            client.update_profile(profile_id, settings=agent_settings)
            print(f"✅ Updated existing ANUBIX profile (ID: {profile_id})")
        else:
            profile = client.create_profile(name="ANUBIX", settings=agent_settings)
            profile_id = profile['id']
            print(f"✅ Created new ANUBIX profile (ID: {profile_id})")
    except Exception as e:
        print(f"❌ Error: Failed to create/update profile: {e}")
        sys.exit(1)

    # Clear agent memory for fresh start
    try:
        client.clear_memory("ANUBIX")
        print("✅ Agent memory cleared - ready for deployment")
    except Exception as e:
        print(f"⚠️  Warning: Failed to clear memory: {e}")
        print("   (Profile was still configured successfully)")

    return profile_id


def main():
    """Main configuration workflow."""
    print("=" * 70)
    print("ANUBIX Agent Configuration for OmniLink v1.0.0")
    print("=" * 70)
    print()

    # Check for API key
    api_key = os.environ.get("OMNI_KEY")
    if not api_key:
        print("❌ Error: OMNI_KEY environment variable not set")
        print("Set it with: export OMNI_KEY=\"olink_your_key_here\"")
        sys.exit(1)

    print(f"✅ Found OMNI_KEY (length: {len(api_key)} characters)")

    # Load agent prompt
    agent_prompt = load_agent_prompt()

    # Configure agent
    profile_id = configure_agent(api_key, agent_prompt)

    print()
    print("=" * 70)
    print("CONFIGURATION COMPLETE")
    print("=" * 70)
    print()
    print(f"Agent Name: ANUBIX")
    print(f"Profile ID: {profile_id}")
    print(f"toolCallbackUrl: {tool_callback_url}")
    print(f"Status: Ready for deployment")
    print()
    print("Next steps:")
    print("1. Start the master node: ros2 run anubix_master master_node")
    print("   (This runs the HTTP tool callback server on port 5055)")
    print("2. Open OmniLink web UI: https://www.omnilink-agents.com")
    print("3. Select 'ANUBIX' from agent dropdown")
    print("4. Send test task: 'Check water stress at (3, 5) with robot_id xxx and task_id yyy'")
    print("5. The web UI will automatically POST tool calls to your master node")
    print()
    print("IMPORTANT: The toolCallbackUrl must be reachable from YOUR BROWSER.")
    print("If the master node runs on a remote machine (Jetson), you need either:")
    print("  - SSH port forwarding: ssh -L 5055:localhost:5055 jetson")
    print("  - Or set ANUBIX_CALLBACK_URL to the Jetson's LAN IP")
    print()


if __name__ == "__main__":
    main()
