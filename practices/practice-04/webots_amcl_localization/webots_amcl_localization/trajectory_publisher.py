
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from tf2_ros import Buffer, TransformListener


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')

        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_speed', 0.08)
        self.declare_parameter('radius', 1.5)
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.5)

        self.v = float(self.get_parameter('linear_speed').value)
        self.w = float(self.get_parameter('angular_speed').value)
        self.radius = float(self.get_parameter('radius').value)
        self.cx = float(self.get_parameter('center_x').value)
        self.cy = float(self.get_parameter('center_y').value)

        self.publisher = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.control)
        self.started = False
        self.finished = False
        self.start_angle = None
        self.last_angle = None
        self.total_angle = 0.0

        self.get_logger().info(
            'Ожидание TF map -> base_link_gt. '
            'После появления TF начнётся объезд по окружности.'
        )

    @staticmethod
    def get_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def publish_velocity(self, linear, angular):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.publisher.publish(msg)

    def control(self):
        if self.finished:
            self.publish_velocity(0.0, 0.0)
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link_gt', rclpy.time.Time()
            )
        except Exception:
            self.publish_velocity(0.0, 0.0)
            return

        p = transform.transform.translation
        yaw = self.get_yaw(transform.transform.rotation)

        dx = p.x - self.cx
        dy = p.y - self.cy
        angle = math.atan2(dy, dx)

        if not self.started:
            self.started = True
            self.start_angle = angle
            self.last_angle = angle
            self.get_logger().info(
                f'Старт: x={p.x:.2f}, y={p.y:.2f}'
            )

        delta = math.atan2(
            math.sin(angle - self.last_angle),
            math.cos(angle - self.last_angle)
        )
        self.total_angle += delta
        self.last_angle = angle

        # Один полный круг вокруг центра
        if abs(self.total_angle) >= 2.0 * math.pi:
            self.finished = True
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info('Объезд завершён!')
            return

        # Направление движения по касательной к окружности
        direction = 1.0

        target_yaw = angle + direction * math.pi / 2.0
        yaw_error = math.atan2(
            math.sin(target_yaw - yaw),
            math.cos(target_yaw - yaw)
        )

        if abs(yaw_error) > 0.25:
            self.publish_velocity(0.0, max(-0.5, min(0.5, 1.5 * yaw_error)))
        else:
            self.publish_velocity(self.v, direction * self.w)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_velocity(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
