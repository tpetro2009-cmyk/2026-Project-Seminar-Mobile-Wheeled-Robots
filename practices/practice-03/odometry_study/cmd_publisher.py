#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class CmdPublisher(Node):
    def __init__(self):
        super().__init__('cmd_publisher')
        self.v = self.declare_parameter('linear_x', 0.1).value
        self.w = self.declare_parameter('angular_z', 0.2).value
        rate = self.declare_parameter('rate', 10.0).value
        self.start_delay = self.declare_parameter('start_delay', 1.0).value  # sim seconds
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.ready_at = None
        self.create_timer(1.0 / rate, self.tick)  # node clock = sim time

    def tick(self):
        now = self.get_clock().now()
        if self.ready_at is None:
            if self.buf.can_transform('odom', 'base_link', Time()):
                self.ready_at = now
            return
        if (now - self.ready_at).nanoseconds * 1e-9 < self.start_delay:
            return
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(self.v)
        msg.twist.angular.z = float(self.w)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
