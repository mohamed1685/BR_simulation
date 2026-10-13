#!/usr/bin/env python3
"""Keyboard teleop for the 4-wheel mecanum due_wheel robot.

Base (mecanum, publishes geometry_msgs/Twist on /cmd_vel):
    w / s : forward / backward          (linear.x)
    a / d : strafe left / right         (linear.y)
    q / e : rotate left / right         (angular.z)
    SPACE : stop the base
Due units (publish std_msgs/Float64, rad/s, on /due_wheel/<joint>/cmd_vel):
    i / k : both due units  + / -
    r / f : front due unit  + / -
    t / g : back due unit   + / -
Other:
    x     : stop everything
    [ / ] : slower / faster (scales every speed)
    Ctrl-C: quit (sends a stop first)

Axes are latched and independent, so you can combine them, e.g. press w then a to drive diagonally.
Frame convention (REP-103): +X forward, +Y left, +Z up; positive angular.z turns left.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

LIN_SPEED = 0.30      # m/s   (at scale 1.0)
ANG_SPEED = 1.00      # rad/s
DUE_SPEED = 3.00      # rad/s
DUES = ['front_due_joint', 'back_due_joint']


class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('wheel_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.due_pub = {j: self.create_publisher(
            Float64, f'/due_wheel/{j}/cmd_vel', 10) for j in DUES}
        self.vx = self.vy = self.wz = 0.0       # latched direction: -1, 0, +1
        self.due = {j: 0.0 for j in DUES}       # latched direction: -1, 0, +1
        self.scale = 1.0
        self.create_timer(0.05, self.publish)   # 20 Hz

    def publish(self):
        msg = Twist()
        msg.linear.x = self.vx * LIN_SPEED * self.scale
        msg.linear.y = self.vy * LIN_SPEED * self.scale
        msg.angular.z = self.wz * ANG_SPEED * self.scale
        self.cmd_pub.publish(msg)
        for j, pub in self.due_pub.items():
            pub.publish(Float64(data=self.due[j] * DUE_SPEED * self.scale))

    def stop_base(self):
        self.vx = self.vy = self.wz = 0.0

    def stop_all(self):
        self.stop_base()
        self.due = {j: 0.0 for j in DUES}

    def status(self):
        return (f'\rscale={self.scale:3.1f}  vx={self.vx * LIN_SPEED * self.scale:+.2f} '
                f'vy={self.vy * LIN_SPEED * self.scale:+.2f} '
                f'wz={self.wz * ANG_SPEED * self.scale:+.2f}  '
                f'due(front,back)=({self.due[DUES[0]] * DUE_SPEED * self.scale:+.1f},'
                f'{self.due[DUES[1]] * DUE_SPEED * self.scale:+.1f})   ')


def main():
    rclpy.init()
    node = MecanumTeleop()

    def set_base(axis, val):
        setattr(node, axis, val)

    def set_due(joints, val):
        for j in joints:
            node.due[j] = val

    keys = {
        'w': lambda: set_base('vx', 1.0), 's': lambda: set_base('vx', -1.0),
        'a': lambda: set_base('vy', 1.0), 'd': lambda: set_base('vy', -1.0),
        'q': lambda: set_base('wz', 1.0), 'e': lambda: set_base('wz', -1.0),
        ' ': node.stop_base, 'x': node.stop_all,
        'i': lambda: set_due(DUES, 1.0), 'k': lambda: set_due(DUES, -1.0),
        'r': lambda: set_due(DUES[:1], 1.0), 'f': lambda: set_due(DUES[:1], -1.0),
        't': lambda: set_due(DUES[1:], 1.0), 'g': lambda: set_due(DUES[1:], -1.0),
    }

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(__doc__)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            c = sys.stdin.read(1)
            if c in keys:
                keys[c]()
            elif c == '[':
                node.scale = max(0.1, node.scale - 0.1)
            elif c == ']':
                node.scale = min(3.0, node.scale + 0.1)
            print(node.status(), end='', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.publish()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print()


if __name__ == '__main__':
    main()
