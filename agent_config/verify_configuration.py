#!/usr/bin/env python3
"""
ANUBIX Configuration Verification Script
==========================================
Verifies that the ANUBIX agent is properly configured on OmniLink platform.

Usage:
    export OMNI_KEY="olink_your_key_here"
    python3 verify_configuration.py
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


def verify_configuration():
    """Verify ANUBIX agent configuration."""
    print("=" * 70)
    print("ANUBIX Configuration Verification")
    print("=" * 70)
    print()

    # Check for API key
    api_key = os.environ.get("OMNI_KEY")
    if not api_key:
        print("❌ Error: OMNI_KEY environment variable not set")
        return False

    print(f"✅ OMNI_KEY found ({len(api_key)} chars)")

    # Connect to OmniLink
    try:
        client = OmniLinkClient(omni_key=api_key)
        print("✅ Connected to OmniLink platform")
    except Exception as e:
        print(f"❌ Failed to connect: {e}")
        return False

    # Check profile exists
    try:
        profiles = client.list_profiles()
        anubix_profile = next((p for p in profiles if p['name'] == 'ANUBIX'), None)

        if not anubix_profile:
            print("❌ ANUBIX profile not found")
            print("   Run: python3 configure_anubix_agent.py")
            return False

        print(f"✅ ANUBIX profile found (ID: {anubix_profile['id']})")

    except Exception as e:
        print(f"❌ Failed to list profiles: {e}")
        return False

    # Verify profile settings
    settings = anubix_profile.get('settings', {})

    checks = [
        ("Agent Name", settings.get('agentName') == 'ANUBIX'),
        ("Tool Use Enabled", settings.get('allowToolUse') == True),
        ("Main Task Present", len(settings.get('mainTask', '')) > 100),
        ("Tools Configured", len(settings.get('availableToolDetails', [])) == 11),
    ]

    print("\nConfiguration Checks:")
    print("-" * 70)

    all_passed = True
    for check_name, passed in checks:
        status = "✅" if passed else "❌"
        print(f"{status} {check_name}")
        if not passed:
            all_passed = False

    if not all_passed:
        print("\n⚠️  Some checks failed - reconfigure with:")
        print("   python3 configure_anubix_agent.py")
        return False

    # Check tool definitions
    tools = settings.get('availableToolDetails', [])
    expected_tools = [
        "supervisor_robot_id",
        "supervisor_task_id",
        "supervisor_nav_vision",
        "supervisor_nav_goal",
        "supervisor_nav_goal_home",
        "supervisor_target_camera",
        "supervisor_perception_goal",
        "supervisor_arm_nav_goal",
        "supervisor_grip",
        "supervisor_spectral_target",
        "supervisor_force_stop",
    ]

    tool_names = [t.get('name') for t in tools]
    missing = set(expected_tools) - set(tool_names)
    extra = set(tool_names) - set(expected_tools)

    if missing:
        print(f"\n❌ Missing tools: {missing}")
        all_passed = False

    if extra:
        print(f"\n⚠️  Extra tools (ignored): {extra}")

    if not missing:
        print(f"\n✅ All 11 supervisor tools properly configured")

    # Test agent responsiveness
    print("\nTesting Agent Communication:")
    print("-" * 70)

    try:
        response = client.chat(
            prompt="Status check - respond with 'READY' if you can see this message",
            agent_name="ANUBIX",
            engine="g1-engine"
        )

        response_text = response.get('text', '').strip()
        print(f"✅ Agent responded: {response_text[:100]}")

        # Check if agent has access to tools
        if 'supervisor' in response_text.lower() or 'tool' in response_text.lower():
            print("✅ Agent appears to be aware of supervisor tools")

    except Exception as e:
        print(f"❌ Agent communication failed: {e}")
        return False

    print("\n" + "=" * 70)
    if all_passed:
        print("✅ CONFIGURATION VERIFIED - ANUBIX READY FOR DEPLOYMENT")
    else:
        print("⚠️  CONFIGURATION HAS ISSUES - PLEASE FIX BEFORE DEPLOYMENT")
    print("=" * 70)

    return all_passed


def main():
    success = verify_configuration()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
