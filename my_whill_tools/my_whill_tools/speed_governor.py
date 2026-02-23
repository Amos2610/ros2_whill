#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class SpeedGovernor(Node):
    def __init__(self):
        super().__init__('speed_governor')
        now = self.get_clock().now()
        self.last_scan_time = now
        self.last_cmd_time = now

        # ---- parameters ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_out_topic', '/whill/controller/cmd_vel')

        self.declare_parameter('front_angle_deg', 25.0)
        self.declare_parameter('stop_dist', 0.5)
        self.declare_parameter('slow_dist', 1.0)
        self.declare_parameter('max_dist', 2.0)
        self.declare_parameter('min_scale', 0.5)
        self.declare_parameter('scale_angular', True)

        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('scan_timeout', 0.2)
        self.declare_parameter('inf_scale', 0.10)
        self.declare_parameter('inf_hold_time', 0.2)

        self.declare_parameter('max_accel', 0.05)          # 線速度の1ループ最大変化量
        self.declare_parameter('max_angular_accel', 0.1)  # 角速度の1ループ最大変化量

        # ---- load params ----
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_in_topic = self.get_parameter('cmd_in_topic').value
        self.cmd_out_topic = self.get_parameter('cmd_out_topic').value

        self.front_angle = math.radians(float(self.get_parameter('front_angle_deg').value))
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.slow_dist = float(self.get_parameter('slow_dist').value)
        self.max_dist = float(self.get_parameter('max_dist').value)
        self.min_scale = float(self.get_parameter('min_scale').value)
        self.scale_angular = bool(self.get_parameter('scale_angular').value)

        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.inf_scale = float(self.get_parameter('inf_scale').value)
        self.inf_hold_time = float(self.get_parameter('inf_hold_time').value)

        self.max_accel = float(self.get_parameter('max_accel').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)

        # ---- state ----
        self.last_scan_min = float('inf')
        self.last_cmd = Twist()
        self.last_cmd_time = now
        self.last_finite_scan_min = float('inf')
        self.last_finite_scan_time = now

        self.prev_out = Twist()  # 前回出力速度保持

        # ---- ROS I/O ----
        self.sub_scan = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.sub_cmd = self.create_subscription(Twist, self.cmd_in_topic, self.on_cmd, 10)
        self.pub_cmd = self.create_publisher(Twist, self.cmd_out_topic, 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.on_timer)

        self.get_logger().info(
            f"started. scan={self.scan_topic}, cmd_in={self.cmd_in_topic}, cmd_out={self.cmd_out_topic}"
        )

    # -------- LaserScan購読 --------
    def on_scan(self, msg: LaserScan):
        now = self.get_clock().now()
        self.last_scan_time = now

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n = len(msg.ranges)
        i_center = int(round((0.0 - angle_min) / angle_inc))
        i_left = int(round(( self.front_angle - angle_min) / angle_inc))
        i_right = int(round((-self.front_angle - angle_min) / angle_inc))
        i1 = max(0, min(n - 1, min(i_left, i_right)))
        i2 = max(0, min(n - 1, max(i_left, i_right)))

        dmin = float('inf')
        for r in msg.ranges[i1:i2 + 1]:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                if r < dmin:
                    dmin = r

        self.last_scan_min = dmin

        if math.isfinite(dmin):
            self.last_finite_scan_min = dmin
            self.last_finite_scan_time = now

    # -------- cmd_vel購読 --------
    def on_cmd(self, msg: Twist):
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    # -------- 距離→スケール --------
    def desired_scale(self, d: float) -> float:
        if d <= self.stop_dist:
            return 0.0
        if d < self.slow_dist:
            t = (d - self.stop_dist) / max(1e-6, (self.slow_dist - self.stop_dist))
            return clamp(t * self.min_scale, 0.0, self.min_scale)
        if d < self.max_dist:
            t = (d - self.slow_dist) / max(1e-6, (self.max_dist - self.slow_dist))
            return clamp(self.min_scale + t * (1.0 - self.min_scale), self.min_scale, 1.0)
        return 1.0

    # -------- 周期処理 --------
    def on_timer(self):
        now = self.get_clock().now()

        # cmd_vel入力途切れ安全停止
        if (now - self.last_cmd_time) > Duration(seconds=self.cmd_timeout):
            return
        # scan途切れ安全停止
        if (now - self.last_scan_time) > Duration(seconds=self.scan_timeout):
            self.pub_cmd.publish(Twist())
            self.prev_out = Twist()
            return

        # 距離スケール計算
        d = self.last_scan_min
        if not math.isfinite(d):
            if math.isfinite(self.last_finite_scan_min) and \
               (now - self.last_finite_scan_time) <= Duration(seconds=self.inf_hold_time):
                d = self.last_finite_scan_min
                s = self.desired_scale(d)
            else:
                s = clamp(self.inf_scale, 0.0, 1.0)
        else:
            s = self.desired_scale(d)

        # 出力速度計算
        out = Twist()
        out.linear.x = s * self.last_cmd.linear.x
        out.linear.y = s * self.last_cmd.linear.y
        out.linear.z = 0.0
        out.angular.z = (s * self.last_cmd.angular.z) if self.scale_angular else self.last_cmd.angular.z

        # --- 線速度加速度制限 ---
        out.linear.x = self.prev_out.linear.x + clamp(out.linear.x - self.prev_out.linear.x,
                                                     -self.max_accel, self.max_accel)
        out.linear.y = self.prev_out.linear.y + clamp(out.linear.y - self.prev_out.linear.y,
                                                     -self.max_accel, self.max_accel)
        # --- 角速度加速度制限 ---
        if self.scale_angular:
            out.angular.z = self.prev_out.angular.z + clamp(out.angular.z - self.prev_out.angular.z,
                                                            -self.max_angular_accel, self.max_angular_accel)

        # 前回出力更新
        self.prev_out = out

        self.pub_cmd.publish(out)


def main():
    rclpy.init()
    node = SpeedGovernor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
