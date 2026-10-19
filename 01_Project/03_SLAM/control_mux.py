#!/usr/bin/env python3
"""Select Nav2 or manual velocity before collision_monitor.

Input: /cmd_vel_nav, /cmd_vel_manual, /control/command, /emergency_stop.
Output: /cmd_vel_muxed (or an explicit test topic). Never opens the chassis serial port.
"""
import argparse
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

sys.path.insert(0, os.path.dirname(__file__))
from control_policy import ControlPolicy, localization_fresh


class ControlMux(Node):
    def __init__(self, nav_topic, manual_topic, out_topic):
        super().__init__('control_mux')
        self.policy = ControlPolicy()
        self.map_odom_stamp = None
        self.last_output = None
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, nav_topic, self.on_nav, 10)
        self.create_subscription(Twist, manual_topic, self.on_manual, 10)
        self.create_subscription(String, '/control/command', self.on_command, 10)
        self.create_subscription(Bool, '/emergency_stop', self.on_emergency, 10)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('control_mux: %s + %s → %s' %
                               (nav_topic, manual_topic, out_topic))

    def on_nav(self, msg):
        self.policy.nav_command(msg.linear.x, msg.angular.z, time.monotonic())

    def on_manual(self, msg):
        self.policy.manual_command(msg.linear.x, msg.angular.z, time.monotonic())

    def on_command(self, msg):
        self.policy.command(msg.data)

    def on_emergency(self, msg):
        if msg.data:
            self.policy.emergency_stop()
            self.get_logger().error('急停锁存：需重启控制链路后恢复')

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id.lstrip('/') == 'map'
                    and transform.child_frame_id.lstrip('/') == 'odom'):
                stamp = transform.header.stamp
                self.map_odom_stamp = stamp.sec + stamp.nanosec * 1e-9

    def tick(self):
        vx, wz = self.policy.output(time.monotonic())
        now_ros = self.get_clock().now().nanoseconds * 1e-9
        if self.policy.mode == 'nav' and not localization_fresh(now_ros, self.map_odom_stamp):
            vx, wz = 0.0, 0.0
            self.get_logger().warn('map→odom 定位 TF 过期：阻止导航速度', throttle_duration_sec=5.0)
        msg = Twist()
        msg.linear.x, msg.angular.z = vx, wz
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nav-topic', default='cmd_vel_nav')
    ap.add_argument('--manual-topic', default='cmd_vel_manual')
    ap.add_argument('--out-topic', default='cmd_vel_muxed')
    args = ap.parse_args()
    rclpy.init()
    node = ControlMux(args.nav_topic, args.manual_topic, args.out_topic)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
