#!/usr/bin/env python3
"""peek_scan —— 一帧雷达快照：正前方 ±90° 内最近障碍 + 三个扇区分布。
用法： python3 ~/RobotCode/01_Project/03_SLAM/peek_scan.py
只订阅 /scan，与导航/建图并行使用，无串口冲突。"""
import math
import rclpy
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    n = rclpy.create_node('peek_scan')
    msg = [None]

    def cb(m):
        msg[0] = m

    n.create_subscription(LaserScan, '/scan', cb, 10)
    t0 = n.get_clock().now()
    while rclpy.ok() and msg[0] is None and (n.get_clock().now() - t0).nanoseconds < 6e9:
        rclpy.spin_once(n, timeout_sec=0.2)
    m = msg[0]
    if m is None:
        print('NO_SCAN（雷达没在跑？）')
        return

    sectors = {'正前(-30°~30°)': [], '左侧(30°~90°)': [], '右侧(-90°~-30°)': []}
    for i, r in enumerate(m.ranges):
        if math.isinf(r) or math.isnan(r) or r < m.range_min or r > m.range_max:
            continue
        a = math.degrees(m.angle_min + i * m.angle_increment)
        if abs(a) > 90:
            continue
        if abs(a) <= 30:
            sectors['正前(-30°~30°)'].append((r, a))
        elif a > 0:
            sectors['左侧(30°~90°)'].append((r, a))
        else:
            sectors['右侧(-90°~-30°)'].append((r, a))

    print('—— 雷达快照（有效距离 ≤4 m）——')
    for name, pts in sectors.items():
        if pts:
            r, a = min(pts)
            print('%s 最近 %.2f m @ %.1f°   (%d 个点)' % (name, r, a, len(pts)))
        else:
            print('%s 无障碍' % name)
    front = sorted(sectors['正前(-30°~30°)'])
    if front:
        print('—— 正前方按角度分布 ——')
        for r, a in front[::max(1, len(front) // 14)]:
            print('   %.1f°  %.2f m' % (a, r))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
