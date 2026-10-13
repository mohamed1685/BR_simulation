#!/usr/bin/env python3
"""Keyboard teleop for the due_wheel robot (publishes joint velocities in rad/s).

  w / s : drive rollers forward / backward   (front_normal + back_normal)
  i / k : due legs forward / backward        (front_due + back_due)
  r / f : front due only  +/-        t / g : back due only  +/-
  SPACE or x : stop everything
  [ / ]      : decrease / increase speed
  q          : quit

A POSITIVE velocity drives the robot toward its front (-X of base_link).
Flip a sign in SIGN below if a wheel spins the wrong way.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

JOINTS = ['front_due_joint', 'back_due_joint',
          'back_normal_joint', 'front_normal_joint']
SIGN = {j: 1.0 for j in JOINTS}          # per-joint direction fix


class WheelTeleop(Node):
    def __init__(self):
        super().__init__('wheel_teleop')
        self.pubs = {j: self.create_publisher(
            Float64, f'/due_wheel/{j}/cmd_vel', 10) for j in JOINTS}
        self.cmd = {j: 0.0 for j in JOINTS}
        self.speed = 15.0                 # rad/s
        self.create_timer(0.05, self.publish)   # 20 Hz

    def publish(self):
        for j, pub in self.pubs.items():
            pub.publish(Float64(data=SIGN[j] * self.cmd[j]))

    def set(self, joints, direction):
        for j in joints:
            self.cmd[j] = direction * self.speed

    def stop(self):
        self.cmd = {j: 0.0 for j in JOINTS}


def main():
    rclpy.init()
    node = WheelTeleop()
    rollers = ['front_normal_joint', 'back_normal_joint']
    dues = ['front_due_joint', 'back_due_joint']
    keys = {
        'w': (rollers, 1), 's': (rollers, -1),
        'i': (dues, 1), 'k': (dues, -1),
        'r': (['front_due_joint'], 1), 'f': (['front_due_joint'], -1),
        't': (['back_due_joint'], 1), 'g': (['back_due_joint'], -1),
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
                node.set(*keys[c])
            elif c in (' ', 'x'):
                node.stop()
            elif c == '[':
                node.speed = max(0.5, node.speed - 0.5)
            elif c == ']':
                node.speed = min(30.0, node.speed + 0.5)
            elif c == 'q':
                break
            print(f'\rspeed={node.speed:4.1f} rad/s  cmd={node.cmd}   ', end='')
    finally:
        node.stop()
        node.publish()
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
