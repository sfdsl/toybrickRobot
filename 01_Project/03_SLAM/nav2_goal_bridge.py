#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav2 目标桥接：把 `/goal_pose` 话题转成 nav2 的 NavigateToPose action

为什么需要它：
    手机网页（phone_teleop.py）和命令行都是往 `/goal_pose` 发 `PoseStamped`；
    而 nav2 只认 **action**（`navigate_to_pose`），话题目标是不会触发导航的。
    本节点做这层桥接 —— **手机页面零改动**，点一下照样走。

行为（对齐自研 goal_nav.py 的语义）：
    · 收到新目标 → 导航过去
    · 行驶中再收新目标 → 放弃当前、立刻改道（cancelTask + 重新 goToPose）
    · 收到 /emergency_stop(True)（来自 safety_watchdog）→ 取消导航 + 发零速

⚠ 实现要点：BasicNavigator.goToPose()/cancelTask() 内部会
   `rclpy.spin_until_future_complete()`，**不能在订阅/定时器回调里调用**（会嵌套 spin 死锁）。
   所以回调只置标志，真正的发送放在主循环里。

用法：
    python3 nav2_goal_bridge.py                                 # 默认 /goal_pose → nav2
    python3 nav2_goal_bridge.py --goal-topic /test_goal_pose    # 隔离自测，不碰真机
"""
import argparse
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from nav2_simple_commander.robot_navigator import BasicNavigator

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # 能 import 到同目录的 goal_nav


class GoalBridge(BasicNavigator):
    """既是 nav2 的 BasicNavigator，又订阅 /goal_pose。"""

    def __init__(self, goal_topic, cmd_topic, estop_topic,
                 map_prefix=None, snap=True, inflation=0.15):
        super().__init__('nav2_goal_bridge')
        self.cmd_topic = cmd_topic
        self.pending_goal = None
        self.estop = False
        self.active = False
        self.snap = snap
        self.gn = None

        if snap:
            import goal_nav as gn          # 复用自研的地图读取与"目标点净化"，不重复实现
            self.gn = gn
            self.m = gn.load_map(map_prefix)
            self.trav = gn.build_traversable(self.m, inflation=inflation,
                                             allow_unknown=False)

        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        self.create_subscription(Bool, estop_topic, self._on_estop, 10)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)

        self.get_logger().info(
            'nav2 目标桥接就绪：%s → navigate_to_pose ｜ 零速 %s ｜ 急停 %s ｜ 目标点净化 %s'
            % (goal_topic, cmd_topic, estop_topic,
               ('开（地图 %s，膨胀 %.2f m）' % (map_prefix, inflation)) if snap else '关'))

    # ---------- 回调（只置标志，不调 nav2 接口） ----------
    def _on_goal(self, msg):
        self.pending_goal = msg
        self.get_logger().info('收到目标点 (%.2f, %.2f) frame=%s'
                               % (msg.pose.position.x, msg.pose.position.y,
                                  msg.header.frame_id))

    def _on_estop(self, msg):
        if msg.data:
            self.estop = True

    def purify(self, goal):
        """目标点净化：贴障碍的目标先吸附到膨胀外的可通行点（与自研 goal_nav.plan 同口径）

        P1 实测：nav2 自带的终点吸附只保证"不压到致命格"，会把不可达的目标停在
        **离真实障碍 0.110 m** 处（< 车半宽 0.12）——车真开过去会被 collision_monitor
        拦在 0.15 m，于是永远"到不了目标"（goal checker 判不到）。
        自研的吸附基于**膨胀后**的可通行区（离障 ≥ 0.15 m），车真能停进去。
        所以下发前用自研同一个函数 `snap_goal_px()` 净化一遍。
        """
        if not self.snap:
            return goal
        g = self.gn.world_to_px(self.m, goal.pose.position.x, goal.pose.position.y)
        if not (0 <= g[0] < self.m['w'] and 0 <= g[1] < self.m['h']):
            self.get_logger().warn('目标点在地图范围外 → 原样下发')
            return goal
        snapped, w = self.gn.snap_goal_px(self.m, self.trav, g)
        if snapped is None:
            self.get_logger().warn('目标点 %.2f m 内无可通行格 → 原样下发（nav2 大概率到不了）'
                                   % 0.5)
            return goal
        if w is None:
            return goal                      # 本来就可通行，不动
        self.get_logger().info('目标点贴障碍 → 吸附到 (%.2f, %.2f)：离障 ≥ 膨胀半径，车停得进去' % w)
        goal.pose.position.x, goal.pose.position.y = w[0], w[1]
        return goal

    # ---------- 主循环里调用 ----------
    def stop_now(self):
        """连发几帧零速度（底盘看门狗 0.5s，多来几帧保险）"""
        for _ in range(5):
            self.cmd_pub.publish(Twist())

    def cancel(self):
        try:
            self.cancelTask()
        except Exception as e:          # 没有正在跑的任务时会抛
            self.get_logger().debug('取消任务：%s' % e)
        self.active = False


def main():
    ap = argparse.ArgumentParser(description='nav2 /goal_pose 桥接')
    ap.add_argument('--goal-topic', default='/goal_pose',
                    help='目标点话题（自测时用 /test_goal_pose 隔离）')
    ap.add_argument('--cmd-topic', default='/cmd_vel', help='发零速度用的话题')
    ap.add_argument('--estop-topic', default='/emergency_stop', help='急停话题')
    ap.add_argument('--map', default=os.path.expanduser('~/RobotCode/04_map/robot_map'),
                    help='目标点净化用的地图前缀（默认 ~/RobotCode/04_map/robot_map）')
    ap.add_argument('--no-snap', action='store_true',
                    help='关掉目标点净化（直接用原始目标点，nav2 可能停在贴墙处）')
    ap.add_argument('--inflation', type=float, default=0.15,
                    help='净化用的膨胀半径 m（默认 0.15，与自研一致）')
    args = ap.parse_args()

    rclpy.init()
    nav = GoalBridge(args.goal_topic, args.cmd_topic, args.estop_topic,
                     map_prefix=args.map, snap=not args.no_snap,
                     inflation=args.inflation)

    try:
        while rclpy.ok():
            rclpy.spin_once(nav, timeout_sec=0.1)

            # 1) 急停优先
            if nav.estop:
                nav.estop = False
                nav.get_logger().error('⚠ 急停：取消导航并停车')
                nav.cancel()
                nav.stop_now()

            # 2) 新目标（改道）
            if nav.pending_goal is not None:
                goal = nav.pending_goal
                nav.pending_goal = None
                goal = nav.purify(goal)      # 贴障碍的目标先吸附（自研同口径）
                if nav.active:
                    nav.get_logger().info('行驶中收到新目标 → 改道')
                    nav.cancel()
                if nav.goToPose(goal):
                    nav.active = True
                else:
                    nav.get_logger().error('目标被 nav2 拒绝')

            # 3) 任务完成
            if nav.active and nav.isTaskComplete():
                nav.active = False
                res = nav.getFeedback()      # 可能 None，仅用于日志
                nav.get_logger().info('✓ 导航任务结束，等待下一个目标'
                                      + ('' if res is None else ''))
    except KeyboardInterrupt:
        pass
    finally:
        nav.cancel()
        nav.stop_now()
        nav.destroyNode()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
