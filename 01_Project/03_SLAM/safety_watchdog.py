#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""安全看门狗：/odom 断流 → 停车；/imu/data 断流 → 仅告警

为什么需要它：
    nav2 **不管**"/odom 断流"（串口假死）这种情况，而自研 N3 安全层里有这条判据
    （>1.5s 没 /odom 就停车退出）。换 nav2 不能把这条安全底线丢掉，所以单做一个小节点：
        · 超过 --timeout（默认 1.5 s）收不到 /odom → 发零速度 + 发布 /emergency_stop(True)
        · /odom 恢复 → 打印恢复信息（不自动恢复导航，由人决定）
        · **首帧给 3 s 宽限**（DDS 发现通常要 0.3~1 s）：宽限内不判，超过仍无 /odom 才急停。
          旧实现从 __init__ 起算，2026-10-19 实测启动时误报过一次 `/odom 断流 1.6s`。

    /emergency_stop 由 nav2_goal_bridge 订阅 → 它会 cancelTask() 取消导航。

判据：底盘正常时 /odom 是 20 Hz 持续发布，空闲也在发 → 不会误触发。

IMU 监控（2026-10-19 补，卡-02 遗留项）：
    背景：IMU 假死时 EKF **完全不吭声** —— `ekf_node` 日志 0 行；而 launch 只能发现
          "进程死了"，发现不了"进程活着但数据不来"（I²C 卡住），后者恰恰更常见。
          那样导航会带着退化的里程计一路跑，没人知道。
    做法：/imu/data 超过 --imu-timeout（默认 1.0 s）没更新 → **只告警，不停车**。
          IMU 丢了 EKF 仍能用轮速继续工作，中途停车反而更危险；要的是让人知道。
          恢复时打一条 INFO。
    ⚠️ 只在"曾经收到过 IMU"之后才监控：`ekf:=false` 的栈里本来就没有 IMU，不该刷假告警。
       若启动后 5 s 一直没见到 /imu/data，会打一条提示说明"监控未启用"。

用法：
    python3 safety_watchdog.py                            # /odom 1.5s 停车，/imu 1.0s 告警
    python3 safety_watchdog.py --timeout 2.0 --dry-run    # 只告警不发车（配合其他自测）
    python3 safety_watchdog.py --imu-timeout 0            # 关掉 IMU 监控
"""
import argparse
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

IMU_ALARM_REMIND = 30.0   # IMU 断流告警的重复提醒间隔（秒）——避免一条刷过去没人看到
IMU_FIRST_WAIT = 5.0      # 启动后等多久才提示"从没见过 IMU"
ODOM_FIRST_GRACE = 3.0    # 首帧 /odom 宽限期（秒）：DDS 发现通常要 0.3~1s，给足余量


class SafetyWatchdog(Node):
    def __init__(self, timeout, dry_run, imu_timeout):
        super().__init__('safety_watchdog')
        self.timeout = timeout
        self.dry_run = dry_run
        self.imu_timeout = imu_timeout
        self.last_odom_t = None     # None = 还没收到过 /odom（与"断流"区分，见 _tick_odom）
        self.alarmed = False
        self.last_estop_t = 0.0

        self.last_imu_t = None      # None = 还没收到过 IMU
        self.imu_alarmed = False
        self.last_imu_alarm_t = 0.0
        self.t_start = time.time()

        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_subscription(Imu, 'imu/data', self._on_imu, 20)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.create_timer(0.2, self._tick)

        self.get_logger().info(
            '安全看门狗就绪：/odom 断流 >%.1fs → 停车%s ｜ /imu/data 断流 >%s → 仅告警'
            % (timeout,
               '（dry-run：只告警不发零速）' if dry_run else '',
               ('%.1fs' % imu_timeout) if imu_timeout > 0 else '（已关闭）'))

    def _on_odom(self, _msg):
        was_none = self.last_odom_t is None
        self.last_odom_t = time.time()
        if self.alarmed:
            self.alarmed = False
            self.estop_pub.publish(Bool(data=False))   # 解除急停标志（桥接只看 True）
            self.get_logger().info(
                '✓ /odom 开始接收' if was_none
                else '✓ /odom 已恢复（导航不会自动恢复，需重新下发目标）')

    def _on_imu(self, _msg):
        was_none = self.last_imu_t is None
        self.last_imu_t = time.time()
        if self.imu_alarmed:
            self.imu_alarmed = False
            self.get_logger().info(
                '✓ /imu/data 开始接收（IMU 监控已启用）' if was_none
                else '✓ /imu/data 已恢复（EKF 重新融合陀螺）')

    def _tick(self):
        self._tick_odom()
        self._tick_imu()

    # ---------------- /odom：断流即停车 ----------------
    def _tick_odom(self):
        now = time.time()
        if self.last_odom_t is None:
            # 还没收到第一帧：可能只是 DDS 发现没完成，也可能底盘真的没起来
            waited = now - self.t_start
            if waited <= ODOM_FIRST_GRACE:
                return      # 宽限期内不判 —— 旧实现从 __init__ 起算，实测启动时误报过一次
            why = ('启动 %.1fs 仍没收到 /odom（已给 %.0fs 发现宽限）'
                   % (waited, ODOM_FIRST_GRACE))
        else:
            gap = now - self.last_odom_t
            if gap <= self.timeout:
                return
            why = '/odom 断流 %.1fs（>%.1fs）' % (gap, self.timeout)

        if not self.alarmed:
            self.alarmed = True
            self.get_logger().error('⚠ %s → 急停并取消导航' % why)
        if not self.dry_run:
            for _ in range(3):
                self.cmd_pub.publish(Twist())
        # 急停只"进入时 + 之后每 5s"发一次：旧实现 0.2s 一发，桥接跟着每 200ms
        # 打印一次"急停"并 cancelTask，日志被刷爆（实测 700ms 内 5 条），刷掉真正有用的信息
        if now - self.last_estop_t >= 5.0:
            self.last_estop_t = now
            self.estop_pub.publish(Bool(data=True))

    # ---------------- /imu/data：断流只告警，不停车 ----------------
    def _tick_imu(self):
        if self.imu_timeout <= 0.0:
            return
        now = time.time()

        if self.last_imu_t is None:
            # 从没见过 IMU：可能是 ekf:=false 的栈，不该刷假告警 → 只提示一次
            if now - self.t_start > IMU_FIRST_WAIT and not self.imu_alarmed:
                self.imu_alarmed = True
                self.get_logger().warn(
                    '未收到过 /imu/data（已等 %.0fs）→ IMU 监控未启用；'
                    '若本栈本该有 IMU，说明 imu_driver 没起来' % IMU_FIRST_WAIT)
            return

        gap = now - self.last_imu_t
        if gap <= self.imu_timeout:
            return
        # 进入告警时打一条，之后每 30s 提醒一次（恢复由 _on_imu 负责清标志）
        if not self.imu_alarmed or now - self.last_imu_alarm_t >= IMU_ALARM_REMIND:
            first = not self.imu_alarmed
            self.imu_alarmed = True
            self.last_imu_alarm_t = now
            self.get_logger().warn(
                '%s/imu/data 断流 %.1fs（>%.1fs）→ EKF 退化为纯轮速、航向无陀螺校正；'
                '**不停车**，但定位会逐步变差'
                % ('⚠ ' if first else '⚠ （仍未恢复）', gap, self.imu_timeout))


def main():
    ap = argparse.ArgumentParser(description='nav2 安全看门狗（/odom 断流停车 + /imu 断流告警）')
    ap.add_argument('--timeout', type=float, default=1.5, help='/odom 断流判定时长（秒）')
    ap.add_argument('--dry-run', action='store_true', help='只告警，不发零速度')
    ap.add_argument('--imu-timeout', type=float, default=1.0,
                    help='/imu/data 断流判定时长（秒）；0 = 关闭监控。只告警、不停车')
    args = ap.parse_args()

    rclpy.init()
    node = SafetyWatchdog(args.timeout, args.dry_run, args.imu_timeout)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # 被 kill / Ctrl-C 时 spin 会抛 ExternalShutdownException，原来只捕 KeyboardInterrupt
        # → 关闭时打一屏 traceback（实测踩到）
        pass
    finally:
        node.destroy_node()          # ROS 2 是 destroy_node()，原写成 ROS 1 的 destroyNode()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
