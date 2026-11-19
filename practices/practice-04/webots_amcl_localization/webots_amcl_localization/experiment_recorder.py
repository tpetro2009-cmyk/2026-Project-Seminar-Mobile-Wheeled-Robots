
import csv
import math
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import Buffer, TransformListener


class ExperimentRecorder(Node):
    def __init__(self):
        super().__init__('experiment_recorder')

        self.declare_parameter('output', 'results/experiment.csv')
        self.declare_parameter('position_threshold', 0.15)
        self.declare_parameter('yaw_threshold', 0.15)
        self.declare_parameter('confirmation_duration', 3.0)

        self.output = self.get_parameter('output').value
        self.position_threshold = float(
            self.get_parameter('position_threshold').value)
        self.yaw_threshold = float(
            self.get_parameter('yaw_threshold').value)
        self.confirmation_duration = float(
            self.get_parameter('confirmation_duration').value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        self.initialized = False
        self.start_time = None
        self.convergence_start = None
        self.convergence_time = None

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        os.makedirs(os.path.dirname(self.output) or '.', exist_ok=True)
        self.file = open(self.output, 'w', newline='')
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            'time',
            'gt_x', 'gt_y', 'gt_yaw',
            'amcl_x', 'amcl_y', 'amcl_yaw',
            'position_error', 'yaw_error'
        ])

        self.timer = self.create_timer(0.1, self.record)

    def amcl_pose_callback(self, msg):
        if not self.initialized:
            self.initialized = True
            self.start_time = (
                self.get_clock().now().nanoseconds / 1e9
            )
            self.get_logger().info(
                'Получена первая оценка AMCL. Начинаем эксперимент.'
            )

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    @staticmethod
    def angle_error(a, b):
        return abs(math.atan2(
            math.sin(a - b),
            math.cos(a - b)
        ))

    def record(self):
        if not self.initialized:
            return

        try:
            gt = self.buffer.lookup_transform(
                'map', 'base_link_gt', Time())
            amcl = self.buffer.lookup_transform(
                'map', 'base_link', Time())
        except Exception:
            return

        gt_p = gt.transform.translation
        amcl_p = amcl.transform.translation

        gt_yaw = self.yaw_from_quaternion(gt.transform.rotation)
        amcl_yaw = self.yaw_from_quaternion(amcl.transform.rotation)

        position_error = math.hypot(
            gt_p.x - amcl_p.x,
            gt_p.y - amcl_p.y
        )
        yaw_error = self.angle_error(gt_yaw, amcl_yaw)

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.start_time

        self.writer.writerow([
            elapsed,
            gt_p.x, gt_p.y, gt_yaw,
            amcl_p.x, amcl_p.y, amcl_yaw,
            position_error, yaw_error
        ])
        self.file.flush()

        within_threshold = (
            position_error <= self.position_threshold
            and yaw_error <= self.yaw_threshold
        )

        if within_threshold:
            if self.convergence_start is None:
                self.convergence_start = elapsed

            if (
                self.convergence_time is None
                and elapsed - self.convergence_start
                >= self.confirmation_duration
            ):
                self.convergence_time = self.convergence_start
                self.get_logger().info(
                    f'Localization converged at '
                    f'{self.convergence_time:.2f}s'
                )
        else:
            self.convergence_start = None

    def destroy_node(self):
        if self.convergence_time is not None:
            self.get_logger().info(
                f'Итоговое время сходимости: '
                f'{self.convergence_time:.2f} с'
            )
        else:
            self.get_logger().info(
                'За время эксперимента сходимость не достигнута.'
            )

        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
