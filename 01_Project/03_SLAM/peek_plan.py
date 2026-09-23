#!/usr/bin/env python3
"""peek_plan —— 抓一帧全局路径，看是否非空、绕行幅度。
用法： python3 ~/RobotCode/01_Project/03_SLAM/peek_plan.py"""
import math
import rclpy
from nav_msgs.msg import Path


def main():
    rclpy.init()
    n = rclpy.create_node('peek_plan')
    msg = [None]

    def cb(m):
        msg[0] = m

    n.create_subscription(Path, '/plan', cb, 10)
    t0 = n.get_clock().now()
    while rclpy.ok() and msg[0] is None and (n.get_clock().now() - t0).nanoseconds < 8e9:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = msg[0]
    if m is None:
        print('NO_PLAN（planner 没在发 /plan，或 BT 没在跑导航）')
        return
    pts = m.poses
    print('plan pts=%d frame=%s stamp=%.2f' % (len(pts), m.header.frame_id,
                                              m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
    if len(pts) == 0:
        print('⚠ 路径为空！planner 返回 0 点 → controller 必然报 0 poses')
        return
    step = max(1, len(pts) // 12)
    for i in range(0, len(pts), step):
        p = pts[i].pose.position
        print('  [%2d] (%.2f, %.2f)' % (i, p.x, p.y))
    ys = [p.pose.position.y for p in pts]
    xs = [p.pose.position.x for p in pts]
    print('x 范围 %.2f~%.2f  y 范围 %.2f~%.2f  绕行幅度 %.2f m'
          % (min(xs), max(xs), min(ys), max(ys), max(ys) - min(ys)))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
