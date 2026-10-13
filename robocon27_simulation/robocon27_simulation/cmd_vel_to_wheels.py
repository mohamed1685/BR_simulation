#!/usr/bin/env python3
"""Map geometry_msgs/Twist on /cmd_vel to the four wheel joint commands.

  linear.x  [m/s]   -> both rollers  (omega = linear.x / roller_radius)
  angular.z [rad/s] -> both due legs (rad/s, directly)

Lets you use joystick_teleop / teleop_twist_keyboard / Nav2 etc.
(All four joints share one axis, so the robot has no steering: it can only
drive forward/back, plus rotate the due legs for climbing.)
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class CmdVelToWheels(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_wheels')
        self.declare_parameter('roller_radius', 0.0636)
        self.declare_parameter('timeout', 0.5)     # stop if /cmd_vel goes quiet
        self.r = self.get_parameter('roller_radius').value
        self.timeout = self.get_parameter('timeout').value
        mk = lambda j: self.create_publisher(
            Float64, f'/due_wheel/{j}/cmd_vel', 10)
        self.rollers = [mk('front_normal_joint'), mk('back_normal_joint')]
        self.dues = [mk('front_due_joint'), mk('back_due_joint')]
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.last = self.get_clock().now()
        self.roller_w = 0.0
        self.due_w = 0.0
        self.create_timer(0.05, self.tick)

    def on_cmd(self, msg):
        self.roller_w = msg.linear.x / self.r
        self.due_w = msg.angular.z
        self.last = self.get_clock().now()

    def tick(self):
        if (self.get_clock().now() - self.last).nanoseconds * 1e-9 > self.timeout:
            self.roller_w = self.due_w = 0.0
        for p in self.rollers:
            p.publish(Float64(data=self.roller_w))
        for p in self.dues:
            p.publish(Float64(data=self.due_w))


def main():
    rclpy.init()
    rclpy.spin(CmdVelToWheels())


if __name__ == '__main__':
    main()
