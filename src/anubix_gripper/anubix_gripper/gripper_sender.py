#!/usr/bin/env python3
"""
Gripper sender — Interactive CLI for testing the gripper.

Usage:
    ros2 run anubix_gripper gripper_sender
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from std_srvs.srv import Trigger


HELP = """
  pick     - ultra-gentle leaf pick (with retries)
  open     - fully open gripper
  close    - fully close gripper
  release  - gently release held leaf
  stop     - emergency stop
  status   - show current position & state
  watch    - toggle live status feed on/off
  help     - show this menu
  quit     - safely exit
"""


class GripperSender(Node):

    def __init__(self):
        super().__init__('gripper_sender')

        self._watching     = False
        self._last_status  = "N/A"
        self._last_pos     = -1.0

        self.cmd_pub = self.create_publisher(String, '/gripper/command', 10)

        self.create_subscription(String,  '/gripper/status',   self._status_callback,   10)
        self.create_subscription(Float32, '/gripper/position', self._position_callback, 10)

        self.pick_client    = self.create_client(Trigger, '/gripper/pick')
        self.open_client    = self.create_client(Trigger, '/gripper/open')
        self.close_client   = self.create_client(Trigger, '/gripper/close')
        self.release_client = self.create_client(Trigger, '/gripper/release')

    def _status_callback(self, msg):
        self._last_status = msg.data
        if self._watching:
            print(f'\r  {msg.data}                    ', end='', flush=True)

    def _position_callback(self, msg):
        self._last_pos = msg.data

    def publish(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        print(f'  Sent: "{cmd}"')

    def call_service(self, client, name):
        print(f'  Calling {name}...')
        if not client.wait_for_service(timeout_sec=2.0):
            print(f'  Service {name} not available — is gripper_node running?')
            return False

        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        if future.result() is not None:
            r = future.result()
            icon = 'OK' if r.success else 'FAIL'
            print(f'  [{icon}] {r.message}')
            return r.success
        else:
            print('  Service call timed out or failed')
            return False

    def show_status(self):
        print(f'  Last status  : {self._last_status}')
        print(f'  Last position: {self._last_pos:.0f}/100')


def main(args=None):
    rclpy.init(args=args)
    node = GripperSender()

    print(HELP)
    print('  Tip: Make sure gripper_node is running in another terminal first.\n')

    while True:
        try:
            raw = input('gripper> ').strip().lower()
        except (KeyboardInterrupt, EOFError):
            break

        if not raw:
            continue

        if raw in ('pick', 'grab', 'leaf'):
            print('  Starting gentle leaf pick...')
            node.call_service(node.pick_client, '/gripper/pick')

        elif raw == 'open':
            node.call_service(node.open_client, '/gripper/open')

        elif raw == 'close':
            node.call_service(node.close_client, '/gripper/close')

        elif raw in ('release', 'drop', 'free'):
            print('  Releasing gently...')
            node.call_service(node.release_client, '/gripper/release')

        elif raw == 'stop':
            print('  Emergency stop!')
            node.publish('stop')

        elif raw == 'status':
            node.show_status()
            node.publish('status')

        elif raw == 'watch':
            node._watching = not node._watching
            state = 'ON' if node._watching else 'OFF'
            print(f'  Live status feed: {state}')

        elif raw in ('help', '?', 'h'):
            print(HELP)

        elif raw in ('quit', 'exit', 'q'):
            print('  Exiting sender...')
            break

        else:
            node.publish(raw)

        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
