#!/usr/bin/env python3
"""
Ensures all critical controllers are active before SLAM/EKF can work.
Waits for joint_state_broadcaster and drive_controller to become active,
retrying activation if needed.

Run BEFORE starting SLAM:
  python3 ~/robot_workspaces/combined_ws/ensure_controllers.py
"""

import subprocess
import sys
import time


REQUIRED_CONTROLLERS = [
    'joint_state_broadcaster',
    'drive_controller',
]

MAX_RETRIES = 10
RETRY_INTERVAL = 3.0


def run(cmd):
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=15)
    return result.stdout.strip(), result.returncode


def get_controller_states():
    """Returns dict of controller_name -> state."""
    out, rc = run(
        'source /opt/ros/jazzy/setup.bash && '
        'source ~/robot_workspaces/combined_ws/install/setup.bash && '
        'ros2 control list_controllers 2>/dev/null'
    )
    states = {}
    for line in out.split('\n'):
        parts = line.split()
        if len(parts) >= 3:
            states[parts[0]] = parts[-1]  # last word is state
    return states


def activate_controller(name):
    """Try to activate a controller."""
    out, rc = run(
        'source /opt/ros/jazzy/setup.bash && '
        'source ~/robot_workspaces/combined_ws/install/setup.bash && '
        f'ros2 control set_controller_state {name} active 2>&1'
    )
    return 'Successfully' in out


def check_topic(topic, timeout=3):
    """Check if a topic has publishers."""
    out, rc = run(
        'source /opt/ros/jazzy/setup.bash && '
        'source ~/robot_workspaces/combined_ws/install/setup.bash && '
        f'ros2 topic info {topic} 2>/dev/null'
    )
    for line in out.split('\n'):
        if 'Publisher count:' in line:
            count = int(line.split(':')[1].strip())
            return count > 0
    return False


def main():
    print("🔧 Controller Activation Check")
    print("=" * 50)

    for attempt in range(1, MAX_RETRIES + 1):
        print(f"\n--- Attempt {attempt}/{MAX_RETRIES} ---")
        states = get_controller_states()

        if not states:
            print("⏳ Controller manager not ready yet...")
            time.sleep(RETRY_INTERVAL)
            continue

        all_active = True
        for ctrl in REQUIRED_CONTROLLERS:
            state = states.get(ctrl, 'NOT FOUND')
            if state == 'active':
                print(f"  ✅ {ctrl}: {state}")
            else:
                print(f"  ❌ {ctrl}: {state} — activating...")
                if activate_controller(ctrl):
                    print(f"     ✅ Activated {ctrl}")
                else:
                    print(f"     ❌ Failed to activate {ctrl}")
                    all_active = False

        if all_active:
            # Verify data is flowing
            print("\n📡 Checking data flow...")
            time.sleep(2)

            odom_ok = check_topic('/drive_controller/odom')
            js_ok = check_topic('/joint_states')

            print(f"  /joint_states:          {'✅' if js_ok else '❌'}")
            print(f"  /drive_controller/odom: {'✅' if odom_ok else '❌'}")

            if odom_ok and js_ok:
                print("\n✅ ALL CONTROLLERS ACTIVE AND DATA FLOWING!")
                print("   You can now start SLAM.")
                return 0
            else:
                print("\n⚠️  Controllers active but data not flowing yet, retrying...")

        time.sleep(RETRY_INTERVAL)

    print("\n❌ FAILED: Could not activate all controllers after max retries.")
    print("   Try restarting the simulation.")
    return 1


if __name__ == '__main__':
    sys.exit(main())
