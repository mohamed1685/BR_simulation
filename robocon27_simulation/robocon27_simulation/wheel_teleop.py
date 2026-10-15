#!/usr/bin/env python3
"""Hold-to-move keyboard teleop for the 4-wheel mecanum due_wheel robot (uses pynput).

The robot moves ONLY while a key is held down. Release it and that motion stops.
The base keys (w a s d q e) and r/f/t/g are hold-to-move; i / k are latched (press once, stop with x).
Keys can be combined (hold w + a to drive diagonally, w + q to drive while turning).

Base (publishes geometry_msgs/Twist on /cmd_vel):
    w / s : forward / backward
    a / d : strafe left / right
    q / e : rotate left / right
Due units (publish std_msgs/Float64, rad/s, on /due_wheel/<joint>/cmd_vel):
    i / k : both due units  + / -   (press ONCE - they keep turning until you stop them)
    x or SPACE : stop the due units
    r / f : front due unit  + / -   (hold)
    t / g : back due unit   + / -   (hold)
Other:
    [ / ] : slower / faster (press once per step)
    Esc   : quit

Install once:  pip install pynput
Note: pynput reads the keyboard globally (any window), and needs an X11 desktop session
(it does not work over plain SSH or on a pure-Wayland session).
"""
import threading

import rclpy
from geometry_msgs.msg import Twist
from pynput import keyboard
from rclpy.node import Node
from std_msgs.msg import Float64

LIN_SPEED = 0.50     # m/s   (at scale 1.0)
ANG_SPEED = 1.00      # rad/s
DUE_SPEED = 3.00      # rad/s
DUES = ['front_due_joint', 'back_due_joint']


class HoldTeleop(Node):
    def __init__(self):
        super().__init__('wheel_teleop')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.due_pub = {j: self.create_publisher(
            Float64, f'/due_wheel/{j}/cmd_vel', 10) for j in DUES}
        self.pressed = set()
        self.due_latched = 0.0                          # -1, 0, +1 : set by i / k, cleared by x / space
        self.lock = threading.Lock()
        self.scale = 1.0
        self.quit = False
        self.create_timer(0.05, self.publish)           # 20 Hz

    # --- keyboard callbacks (run in pynput's thread) ---
    @staticmethod
    def _name(key):
        if key == keyboard.Key.esc:
            return 'esc'
        if key == keyboard.Key.space:
            return ' '
        return key.char.lower() if getattr(key, 'char', None) else None

    def on_press(self, key):
        name = self._name(key)
        if name is None:
            return
        with self.lock:
            if name == 'esc':
                self.quit = True
            elif name in self.pressed:
                return                                  # auto-repeat of a held key
            elif name == 'i':
                self.due_latched = 1.0
                print('due units: forward (x or space to stop)')
            elif name == 'k':
                self.due_latched = -1.0
                print('due units: backward (x or space to stop)')
            elif name in ('x', ' '):
                self.due_latched = 0.0
                print('due units: stopped')
            elif name == '[':
                self.scale = max(0.1, round(self.scale - 0.1, 1))
            elif name == ']':
                self.scale = min(3.0, round(self.scale + 0.1, 1))
            self.pressed.add(name)

    def on_release(self, key):
        name = self._name(key)
        if name is not None:
            with self.lock:
                self.pressed.discard(name)

    # --- publish whatever is held right now ---
    def axis(self, plus, minus):
        return float(plus in self.pressed) - float(minus in self.pressed)

    def publish(self):
        with self.lock:
            s = self.scale
            msg = Twist()
            msg.linear.x = self.axis('w', 's') * LIN_SPEED * s
            msg.linear.y = self.axis('a', 'd') * LIN_SPEED * s
            msg.angular.z = self.axis('q', 'e') * ANG_SPEED * s
            due = {
                'front_due_joint': self.due_latched + self.axis('r', 'f'),
                'back_due_joint': self.due_latched + self.axis('t', 'g'),
            }
        self.cmd_pub.publish(msg)
        for j, pub in self.due_pub.items():
            d = max(-1.0, min(1.0, due[j]))
            pub.publish(Float64(data=d * DUE_SPEED * s))


def main():
    rclpy.init()
    node = HoldTeleop()
    print(__doc__)
    listener = keyboard.Listener(on_press=node.on_press, on_release=node.on_release)
    listener.start()
    try:
        while rclpy.ok() and not node.quit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        node.pressed.clear()
        node.due_latched = 0.0
        node.publish()                                  # final all-zero command
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
