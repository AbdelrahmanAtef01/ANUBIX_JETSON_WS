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
    prompt_path = r"ANUBIX_AGENT_PROMPT_v2.txt"

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

    # Agent settings
    agent_settings = {
        "agentName": "ANUBIX",
        "mainTask": agent_prompt,
        "agentPersonality": "Professional",
        "allowToolUse": False,
        "availableCommands": "",
        "availableTools": "",
        "availableToolDetails": [],
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
    print(f"Status: Ready for deployment")
    print()
    print("Home positions configured in ROS param files:")
    print("  - Navigation home: src/anubix_navigation/config/nav_params.yaml")
    print("  - Arm home: src/anubix_arm/config/arm_params.yaml")
    print()
    print("Next steps:")
    print("1. Open OmniLink web UI: https://www.omnilink-agents.com")
    print("2. Select 'ANUBIX' from agent dropdown")
    print("3. Send test task: 'Check water stress at (3, 5) with robot_id xxx and task_id yyy'")
    print("4. Verify 11-step execution completes successfully")
    print()


if __name__ == "__main__":
    main()
