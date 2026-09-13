#!/usr/bin/env python3
"""One-key emergency stop for the robot dog.

Press 'q' (or Ctrl+C) at any time -> immediately:
  1. disables motion on custom_nav (/set_motion_enabled false)
  2. triggers the SDK soft e-stop (/zs1w/passive -> damped lie down)
  3. publishes zero /cmd_vel
Run this in a dedicated terminal and keep it focused during any motion test.
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Twist

HELP = """
=== EMERGENCY STOP READY ===
Press 'q' or Ctrl+C to EMERGENCY STOP.
The robot will damp and lie down immediately.
"""


class EStop(Node):
    def __init__(self):
        super().__init__("e_stop")
        self.passive_cli = self.create_client(Trigger, "/zs1w/passive")
        self.motion_cli = self.create_client(SetBool, "/set_motion_enabled")
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(HELP)

    def call(self, cli, req):
        if cli.wait_for_service(timeout_sec=1.0):
            cli.call_async(req)

    def trigger(self):
        self.get_logger().warn("!!! EMERGENCY STOP TRIGGERED !!!")
        # 1. disable motion in custom_nav
        self.call(self.motion_cli, SetBool.Request(data=False))
        # 2. zero cmd_vel
        for _ in range(3):
            self.vel_pub.publish(Twist())
        # 3. SDK soft e-stop (damped lie down)
        self.call(self.passive_cli, Trigger.Request())


def main():
    rclpy.init()
    node = EStop()
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                k = sys.stdin.read(1)
                if k in ("q", "Q"):
                    node.trigger()
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        node.trigger()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
