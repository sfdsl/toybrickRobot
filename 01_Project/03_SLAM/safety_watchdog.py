#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全看门狗：/odom 断流 → 停车 + 通知桥接取消导航

为什么需要它：
    nav2 **不管**"/odom 断流"（串口假死）这种情况，而自研 N3 安全层里有这条判据
    （>1.5s 没 /odom 就停车退出）。换 nav2 不能把这条安全底线丢掉，所以单做一个小节点：
        · 超过 --timeout（默认 1.5 s）收不到 /odom → 发零速度 + 发布 /emergency_stop(True)
        · /odom 恢复 → 打印恢复信息（不自动恢复导航，由人决定）

    /emergency_stop 由 nav2_goal_bridge 订阅 → 它会 cancelTask() 取消导航。

判据：底盘正常时 /odom 是 20 Hz 持续发布，空闲也在发 → 不会误触发。

用法：
    python3 safety_watchdog.py                       # 默认 1.5s
    python3 safety_watchdog.py --timeout 2.0 --dry-run   # 只告警不发车（配合其他自测）
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class SafetyWatchdog(Node):
    def __init__(self, timeout, dry_run):
        super().__init__('safety_watchdog')
        self.timeout = timeout
        self.dry_run = dry_run
        self.last_odom_t = time.time()
        self.alarmed = False

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            '安全看门狗就绪：/odom 断流 >%.1fs → 停车%s'
            % (timeout, '（dry-run：只告警不发零速）' if dry_run else ''))

    def _on_odom(self, _msg):
        self.last_odom_t = time.time()
        if self.alarmed:
            self.alarmed = False
            self.get_logger().info('✓ /odom 已恢复（导航不会自动恢复，需重新下发目标）')

    def _tick(self):
        gap = time.time() - self.last_odom_t
        if gap <= self.timeout:
            return
        if not self.alarmed:
            self.alarmed = True
            self.get_logger().error('⚠ /odom 断流 %.1fs（>%.1fs）→ 急停并取消导航'
                                    % (gap, self.timeout))
        if not self.dry_run:
            for _ in range(3):
                self.cmd_pub.publish(Twist())
        self.estop_pub.publish(Bool(data=True))


def main():
    ap = argparse.ArgumentParser(description='nav2 安全看门狗（/odom 断流保护）')
    ap.add_argument('--timeout', type=float, default=1.5, help='断流判定时长（秒）')
    ap.add_argument('--dry-run', action='store_true', help='只告警，不发零速度')
    args = ap.parse_args()

    rclpy.init()
    node = SafetyWatchdog(args.timeout, args.dry_run)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroyNode()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
