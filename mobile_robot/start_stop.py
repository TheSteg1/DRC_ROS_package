#!/usr/bin/env python3
"""
start_stop.py
--------------------
Minimal keyboard listener that publishes Bool to 'enable_control'.

  s  → start (enable = True)
  x  → stop  (enable = False)
  q  → quit this node

Run this in its own terminal. It grabs raw keypresses (no Enter needed)
using termios, so it must run in an actual terminal (not piped/background).
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Bool


class StartStop(Node):

    def __init__(self):
        super().__init__('start_stop')
        self._pub = self.create_publisher(
            Bool, 'enable_control', QoSPresetProfiles.SYSTEM_DEFAULT.value
        )
        self.get_logger().info(
            "Keyboard trigger ready — press 's' to start, 'x' to stop, 'q' to quit"
        )

    def publish_state(self, enabled: bool):
        msg = Bool()
        msg.data = enabled
        self._pub.publish(msg)
        self.get_logger().info(f"enable_control -> {enabled}")


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = StartStop()

    settings = termios.tcgetattr(sys.stdin)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == 's':
                node.publish_state(True)
            elif key == 'x':
                node.publish_state(False)
            elif key == 'q':
                break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # safety: publish stop on exit
        node.publish_state(False)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()