#!/usr/bin/env python3

import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  
import rclpy  
from geometry_msgs.msg import TwistStamped  
from rclpy.clock import ClockType  
from rclpy.node import Node  
from rclpy.time import Time  
from rosgraph_msgs.msg import Clock  
from tf2_ros import Buffer, TransformException, TransformListener  


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Recorder(Node):
    def __init__(self):
        super().__init__('experiment_recorder')
        self.duration = self.declare_parameter('duration', 130.0).value
        self.period = self.declare_parameter('sample_period', 0.05).value
        self.lag = int(self.declare_parameter('lag', 0.3).value * 1e9)
        self.out_dir = self.declare_parameter('output_dir', 'results').value
        self.name = self.declare_parameter('output_name', 'noise_0').value
        self.noise = self.declare_parameter('noise', 0.0).value

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.on_cmd, 100)

        self.now_ns = 0
        self.cmds = []          
        self.started = False
        self.done = False
        self.t0 = 0
        self.next_t = 0
        self.t_int = 0
        self.cmd_idx = -1
        self.cx = self.cy = self.cth = 0.0
        self.rows = []

    
    def on_cmd(self, msg):
        t = Time.from_msg(msg.header.stamp).nanoseconds or self.now_ns
        self.cmds.append((t, msg.twist.linear.x, msg.twist.angular.z))

    def on_clock(self, msg):
        self.now_ns = msg.clock.sec * 10**9 + msg.clock.nanosec
        if self.done:
            return
        if not self.started:
            self.try_start()
        else:
            self.process()

    
    def pose_at(self, frame, t_ns):
        tr = self.buf.lookup_transform(
            'map', frame, Time(nanoseconds=t_ns, clock_type=ClockType.ROS_TIME))
        p = tr.transform.translation
        return p.x, p.y, yaw_of(tr.transform.rotation)

    def try_start(self):
        if not self.cmds:
            return
        t0 = self.now_ns - self.lag
        try:
            gt = self.pose_at('base_link_gt', t0)
            self.pose_at('base_link', t0)
        except TransformException:
            return
        self.t0 = self.t_int = self.next_t = t0
        self.cx, self.cy, self.cth = gt   
        self.started = True
        self.get_logger().info(
            f'Start at sim t={t0 * 1e-9:.2f}s, pose=({gt[0]:.3f}, {gt[1]:.3f}, {gt[2]:.3f})')

    
    def integrate_to(self, t_end):
        while self.t_int < t_end:
            while (self.cmd_idx + 1 < len(self.cmds)
                   and self.cmds[self.cmd_idx + 1][0] <= self.t_int):
                self.cmd_idx += 1
            if self.cmd_idx >= 0:
                v, w = self.cmds[self.cmd_idx][1:]
            else:
                v = w = 0.0
            t_next = t_end
            if self.cmd_idx + 1 < len(self.cmds):
                t_next = min(t_next, self.cmds[self.cmd_idx + 1][0])
            dt = (t_next - self.t_int) * 1e-9
            if abs(w) > 1e-9:
                self.cx += v / w * (math.sin(self.cth + w * dt) - math.sin(self.cth))
                self.cy += v / w * (-math.cos(self.cth + w * dt) + math.cos(self.cth))
            else:
                self.cx += v * dt * math.cos(self.cth)
                self.cy += v * dt * math.sin(self.cth)
            self.cth = wrap(self.cth + w * dt)
            self.t_int = t_next

    
    def process(self):
        t_avail = self.now_ns - self.lag
        t_stop = self.t0 + int(self.duration * 1e9)
        while self.next_t <= min(t_avail, t_stop):
            try:
                gt = self.pose_at('base_link_gt', self.next_t)
                od = self.pose_at('base_link', self.next_t)
            except TransformException:
                break   
            self.integrate_to(self.next_t)
            self.rows.append((
                (self.next_t - self.t0) * 1e-9, *gt, *od, self.cx, self.cy, self.cth))
            self.next_t += int(self.period * 1e9)
        if self.next_t > t_stop:
            self.finish()


    def finish(self):
        self.done = True
        os.makedirs(self.out_dir, exist_ok=True)
        base = os.path.join(self.out_dir, self.name)
        with open(base + '.csv', 'w', newline='') as f:
            wr = csv.writer(f)
            wr.writerow(['t', 'gt_x', 'gt_y', 'gt_yaw', 'odom_x', 'odom_y', 'odom_yaw',
                         'cmd_x', 'cmd_y', 'cmd_yaw'])
            wr.writerows(self.rows)

        t = [r[0] for r in self.rows]
        gx, gy = [r[1] for r in self.rows], [r[2] for r in self.rows]
        ox, oy = [r[4] for r in self.rows], [r[5] for r in self.rows]
        cx, cy = [r[7] for r in self.rows], [r[8] for r in self.rows]
        pe_o = [math.hypot(r[4] - r[1], r[5] - r[2]) for r in self.rows]
        pe_c = [math.hypot(r[7] - r[1], r[8] - r[2]) for r in self.rows]
        ye_o = [abs(wrap(r[6] - r[3])) for r in self.rows]
        ye_c = [abs(wrap(r[9] - r[3])) for r in self.rows]

        fig = plt.figure(figsize=(14, 6))
        gs = fig.add_gridspec(2, 2, width_ratios=[1, 1])
        ax = fig.add_subplot(gs[:, 0])
        ax.plot(gx, gy, label='GT')
        ax.plot(ox, oy, label='Одометрия по энкодерам')
        ax.plot(cx, cy, '--', label='Одометрия по командам')
        ax.set_aspect('equal', adjustable='datalim')
        ax.set_xlabel('x, м')
        ax.set_ylabel('y, м')
        ax.set_title(f'Траектории (encoderNoise = {self.noise})')
        ax.grid(True)
        ax.legend()
        a1 = fig.add_subplot(gs[0, 1])
        a1.plot(t, pe_o, label='энкодеры')
        a1.plot(t, pe_c, label='команды')
        a1.set_xlabel('Время от начала эксперимента, с')
        a1.set_ylabel('Ошибка положения, м')
        a1.grid(True)
        a1.legend()
        a2 = fig.add_subplot(gs[1, 1])
        a2.plot(t, ye_o, label='энкодеры')
        a2.plot(t, ye_c, label='команды')
        a2.set_xlabel('Время от начала эксперимента, с')
        a2.set_ylabel('Ошибка угла рысканья, рад')
        a2.grid(True)
        a2.legend()
        fig.tight_layout()
        fig.savefig(base + '.png', dpi=130)
        self.get_logger().info(
            f'Saved {base}.png ({len(self.rows)} samples). '
            f'Final pos err: odom={pe_o[-1]:.3f} m, cmd={pe_c[-1]:.3f} m')


def main():
    rclpy.init()
    node = Recorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
