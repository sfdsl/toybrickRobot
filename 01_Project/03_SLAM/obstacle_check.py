#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_check —— 动态避障实车验收：记录碰撞监控事件 + 复盘摘要（P2-B2）

背景（17 号文档 P2「验证动态行人/移动障碍的绕行或等待」）：
    nav2 的局部重规划 + obstacle_layer 已经在跑，B2 **不改算法**，只做实车验收。
    本脚本负责**取证**：把 collision_monitor 的减速/急停事件、实际速度、
    目标下发记录下来，跑完出一份可对照判据的摘要。

监听话题：
    /polygon_slowdown   geometry_msgs/PolygonStamped  减速区被触发
    /polygon_stop       geometry_msgs/PolygonStamped  急停区被触发
    /cmd_vel            geometry_msgs/Twist           实际下发速度
    /goal_pose          geometry_msgs/PoseStamped     目标下发
    /odom               nav_msgs/Odometry             实际速度（对照）

用法：
    python3 obstacle_check.py                # 一直记录，Ctrl-C 出总结
    python3 obstacle_check.py -d 120         # 记 120 秒自动出总结
    python3 obstacle_check.py --csv /tmp/ob.csv   # 同时写事件流水（复盘用）

验收判据（写进卡-09，见 02_doc/04_执行检验/）：
    ① 行人横穿：车应**绕行或减速等待**，不得撞上
    ② 障碍贴脸：急停区触发后车必须**停住**，且解除后能继续任务
    ③ 安全层优先：即使 nav2 路径直穿，collision_monitor 也应阻停
    ④ 事件可复盘：CSV 里能还原"何时何地被触发、持续多久"

复盘流程（跑完之后）：
    1) 看本脚本总结里的「急停次数 / 最长急停时长」
    2) 对照 --bag 录的 rosbag（`robotnav --nav2 --ui --bag`），回放看车的轨迹：
         ros2 bag play <bag> --topics /tf /odom /scan /polygon_slowdown /polygon_stop
    3) 用 mapweb 看车当时在地图上的位置，确认是「绕行」还是「贴脸」
"""
import argparse
import csv
import time

import rclpy
from geometry_msgs.msg import PolygonStamped, PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def _xy(pose):
    return (round(pose.position.x, 3), round(pose.position.y, 3))


class ObstacleCheck(Node):
    def __init__(self, csv_path=None):
        super().__init__('obstacle_check')
        self.csv_path = csv_path
        self.csv = None
        self.writer = None
        self.t0 = time.time()
        self.ev = []                  # 事件流水
        self.slowdowns = 0
        self.stops = 0
        self.stop_spans = []          # 每次急停的持续时长
        self._stop_t0 = None
        self.cmd_max = 0.0
        self.cmd_nonzero = 0
        self.n_cmd = 0
        self.n_odom = 0
        self.goal_n = 0
        self.last_goal = None

        self.create_subscription(PolygonStamped, '/polygon_slowdown',
                                 self.on_slowdown, 10)
        self.create_subscription(PolygonStamped, '/polygon_stop',
                                 self.on_stop, 10)
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    # ---------------- 事件 ----------------
    def _log(self, kind, detail):
        t = time.time() - self.t0
        self.ev.append((t, kind, detail))
        if self.writer:
            self.writer.writerow(['%.3f' % t, kind, detail])
            self.csv.flush()

    def on_slowdown(self, _msg):
        self.slowdowns += 1
        self._log('SLOWDOWN', '减速区触发')

    def on_stop(self, _msg):
        now = time.time()
        if self._stop_t0 is None:
            self._stop_t0 = now
        self.stops += 1
        self._log('STOP', '急停区触发')

    def on_cmd(self, m):
        self.n_cmd += 1
        v = abs(m.linear.x)
        self.cmd_max = max(self.cmd_max, v)
        if v > 1e-3:
            self.cmd_nonzero += 1
        # 急停解除：有非零速度且之前在急停 → 记一段时长
        if self._stop_t0 is not None and v > 1e-3:
            self.stop_spans.append(time.time() - self._stop_t0)
            self._stop_t0 = None
            self._log('RESUME', '急停解除')

    def on_goal(self, m):
        self.goal_n += 1
        self.last_goal = _xy(m.pose)
        self._log('GOAL', '目标 %s' % (self.last_goal,))

    def on_odom(self, _m):
        self.n_odom += 1

    # ---------------- 摘要 ----------------
    def summary(self):
        # 收尾未闭合的急停
        if self._stop_t0 is not None:
            self.stop_spans.append(time.time() - self._stop_t0)
        dur = time.time() - self.t0
        lines = []
        lines.append('=========== 动态避障验收摘要 ===========')
        lines.append('  记录时长        : %.1f s' % dur)
        lines.append('  目标下发        : %d 次  最近 %s'
                     % (self.goal_n, self.last_goal or '—'))
        lines.append('  cmd_vel 帧数    : %d（其中非零 %d）'
                     % (self.n_cmd, self.cmd_nonzero))
        lines.append('  odom 帧数       : %d' % self.n_odom)
        lines.append('  最大下发速度    : %.3f m/s' % self.cmd_max)
        lines.append('')
        lines.append('  ⚡ 减速区触发    : %d 次' % self.slowdowns)
        lines.append('  🛑 急停区触发    : %d 次' % self.stops)
        if self.stop_spans:
            lines.append('     急停持续    : 最长 %.2f s / 平均 %.2f s'
                         % (max(self.stop_spans),
                            sum(self.stop_spans) / len(self.stop_spans)))
        lines.append('')
        lines.append('  判据对照：')
        lines.append('    ① 行人横穿 → 应绕行或减速等待：看 SLOWDOWN 次数与 cmd 速度曲线')
        lines.append('    ② 贴脸障碍 → 急停后必须停住：急停次数 >0 且 RESUME 后能继续')
        lines.append('    ③ 安全层优先 → 即使路径直穿也应阻停：看 STOP 是否在 path 未变时触发')
        lines.append('    ④ 可复盘    → CSV 事件流水 + rosbag')
        lines.append('========================================')
        return '\n'.join(lines)


def main():
    ap = argparse.ArgumentParser(description='动态避障实车验收：记录碰撞监控事件并出摘要')
    ap.add_argument('-d', '--duration', type=float, default=0.0,
                    help='记录时长（秒），0 = 一直记到 Ctrl-C')
    ap.add_argument('--csv', metavar='PATH', help='同时写事件流水 CSV（复盘用）')
    args = ap.parse_args()

    rclpy.init()
    node = ObstacleCheck(args.csv)
    if args.csv:
        node.csv = open(args.csv, 'w', newline='', encoding='utf-8')
        node.writer = csv.writer(node.csv)
        node.writer.writerow(['t_sec', 'event', 'detail'])
    print('开始记录（Ctrl-C 或 -d 到时出总结）...')
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration and time.time() - t0 > args.duration:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print('\n' + node.summary())
        if node.csv:
            node.csv.close()
            print('  事件流水：%s' % args.csv)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
