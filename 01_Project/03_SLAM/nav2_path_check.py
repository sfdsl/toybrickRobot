#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav2 路径校验 —— 与自研 A* **同口径**对比（P1 判据：离真实障碍 ≥ 车半宽）

为什么单独一个脚本：
    自研的 `navcheck`（check_path.py）只能检查**它自己规划出来的**路径；
    nav2 的路径要从 /plan 话题拿，所以需要这个入口。
    判据与量距函数**直接复用** check_path.py（rasterize / min_on_path / verdict），
    保证两条路径量的是同一把尺子：

    ① 真实障碍（未膨胀，pgm==0）  → 物理安全，判据 ≥ --min-clear（0.12 = 车半宽）
    ② 不可通行区（障碍 ∪ 未知）    → 规划器口径，判据 ≥ --inflation（0.15）

用法（需先 source ROS：脚本 import goal_nav，它会 import rclpy）：
    # 1) 先抓 nav2 的路径（另开一个终端或用下面的 --subscribe）
    python3 nav2_path_check.py --subscribe --goal 2.5 -1.0 --save /tmp/nav2_plan.json
    # 2) 校验 + 与自研 A* 对比
    python3 nav2_path_check.py --path-file /tmp/nav2_plan.json --goal 2.5 -1.0
    python3 nav2_path_check.py --path-file /tmp/nav2_plan.json --goal 2.5 -1.0 \
        --start 0 0 --png /tmp/nav2_vs_astar.png

别名建议：nav2check
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goal_nav as gn          # noqa: E402  地图 / A* / 距离图
import check_path as cp        # noqa: E402  同口径的量距与判定


def world_path_to_px(m, pts):
    return [gn.world_to_px(m, float(x), float(y)) for x, y in pts]


def measure(m, d_obs, d_blk, px_path, inflation, min_clear):
    """沿折线量两个口径的最小距离 + 判定（复用 check_path 的函数）"""
    pts = cp.rasterize(px_path, m['w'], m['h'])
    min_obs, px_obs = cp.min_on_path(pts, d_obs)
    min_blk, _ = cp.min_on_path(pts, d_blk)
    v, why = cp.verdict(min_obs, min_blk, inflation, min_clear)
    return {'min_obs': min_obs, 'min_blk': min_blk, 'verdict': v, 'why': why,
            'px': px_obs, 'n_px': len(pts)}


def own_astar_path(m, start_xy, goal_xy, inflation):
    """自研 A*（与真机同一份代码：GoalNav.plan **含目标点吸附** → 抽稀 → 拐角平滑）

    必须走 GoalNav.plan() 而不是直接 astar()：目标点若落在膨胀后的不可通行区，
    自研会吸附到最近的可通行点（nav2 那边对应的是 tolerance），
    直接用 astar() 会因为终点不可通行而返回 None，对比就不公平了。
    """
    blocked, _ = gn.blocked_mask(m, allow_unknown=False)      # 含未知
    trav = gn.build_traversable(m, inflation=inflation, allow_unknown=False)
    cost = gn.clearance_cost(m, blocked, 0.8, 3.0)
    shim = cp._Shim(m, trav, (start_xy[0], start_xy[1], 0.0), blocked, cost)
    path, why = gn.GoalNav.plan(shim, (goal_xy[0], goal_xy[1]))
    if path is None:
        return None, why
    simp, _ = gn.follow_px_path(m, trav, path, 0.1, 2)
    return simp, why


def subscribe_plan(save_path, timeout=40.0):
    """订阅 /plan 一次并保存（nav_msgs/Path → JSON 世界坐标）"""
    import rclpy
    from nav_msgs.msg import Path
    rclpy.init()
    node = rclpy.create_node('nav2_path_capture')
    got = []

    def cb(msg):
        pts = [[round(p.pose.position.x, 4), round(p.pose.position.y, 4)]
               for p in msg.poses]
        json.dump({'frame': msg.header.frame_id, 'points': pts},
                  open(save_path, 'w'))
        got.append(pts)
        print('抓到 /plan：%d 个点（frame=%s）→ %s' % (len(pts), msg.header.frame_id, save_path))

    node.create_subscription(Path, '/plan', cb, 10)
    t = 0.0
    while not got and t < timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
        t += 0.5
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        print('✗ %.0f 秒内没收到 /plan（nav2 起来了吗？目标发了吗？）' % timeout)
        return None
    return got[0]


def main():
    ap = argparse.ArgumentParser(description='nav2 路径校验（与自研 A* 同口径）')
    ap.add_argument('--map', default=gn.MAP_DEFAULT, help='地图前缀（默认 %s）' % gn.MAP_DEFAULT)
    ap.add_argument('--path-file', help='nav2 路径 JSON（{"points": [[x,y], ...]}）')
    ap.add_argument('--subscribe', action='store_true', help='改为订阅 /plan 抓一次')
    ap.add_argument('--save', default='/tmp/nav2_plan.json', help='--subscribe 保存位置')
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='起点（自研 A* 对比用，默认 0 0）')
    ap.add_argument('--goal', nargs=2, type=float, required=True, metavar=('X', 'Y'),
                    help='目标点（自研 A* 对比用）')
    ap.add_argument('--inflation', type=float, default=0.15, help='膨胀半径 m（默认 0.15）')
    ap.add_argument('--min-clear', type=float, default=0.12, help='车半宽 m（默认 0.12）')
    ap.add_argument('--png', help='可选：出对比图（nav2 蓝 / 自研 绿）')
    args = ap.parse_args()

    if args.subscribe:
        subscribe_plan(args.save)
        return 0

    if not args.path_file:
        print('✗ 需要 --path-file 或 --subscribe')
        return 1

    m = gn.load_map(args.map)
    pts, frame = (json.load(open(args.path_file))['points'],
                  json.load(open(args.path_file)).get('frame', '?'))

    blocked_true, _ = gn.blocked_mask(m, allow_unknown=True)        # ① 只算真实障碍
    blocked_all, _ = gn.blocked_mask(m, allow_unknown=False)        # ② 障碍 ∪ 未知
    d_obs = gn.dist_to_obstacles(m, blocked_true)
    d_blk = gn.dist_to_obstacles(m, blocked_all)

    px_nav2 = world_path_to_px(m, pts)
    nav2 = measure(m, d_obs, d_blk, px_nav2, args.inflation, args.min_clear)

    print()
    print('地图 %s｜起点 (%.2f, %.2f) → 目标 (%.2f, %.2f)｜nav2 路径 %d 点（frame=%s）'
          % (os.path.basename(args.map), args.start[0], args.start[1],
             args.goal[0], args.goal[1], len(pts), frame))
    print('-' * 78)
    print('%-10s %-8s %-10s %-10s %-10s %s'
          % ('路径', '点数', '长度 m', '离真实障碍', '离不可通行', '判定'))
    print('%-10s %-8d %-10.2f %-10.3f %-10.3f %s %s'
          % ('nav2', len(pts), gn.path_length_m(m, px_nav2),
             nav2['min_obs'], nav2['min_blk'], nav2['verdict'], nav2['why']))

    px_own, own_why = own_astar_path(m, args.start, args.goal, args.inflation)
    if px_own is None:
        print('%-10s %-8s %-10s %-10s %-10s %s'
              % ('自研A*', '-', '-', '-', '-', '✗ 规划失败：%s' % own_why))
        own = None
    else:
        own = measure(m, d_obs, d_blk, px_own, args.inflation, args.min_clear)
        print('%-10s %-8d %-10.2f %-10.3f %-10.3f %s %s'
              % ('自研A*', len(px_own), gn.path_length_m(m, px_own),
                 own['min_obs'], own['min_blk'], own['verdict'], own['why']))
        print('         自研规划说明：%s' % own_why)
    print('-' * 78)
    print('判据：离真实障碍 ≥ %.2f m（车半宽），离不可通行区 ≥ %.2f m（膨胀半径）'
          % (args.min_clear, args.inflation))

    if args.png and own is not None:
        import cv2
        import numpy as np
        img = np.full(m['pgm'].shape + (3,), 60, np.uint8)
        img[m['pgm'] == 254] = (235, 235, 235)
        img[blocked_true] = (40, 40, 40)
        img[blocked_all & ~blocked_true] = (90, 90, 90)
        def _t(p):
            return (int(p[0]), int(p[1]))

        for i in range(len(px_nav2) - 1):
            cv2.line(img, _t(px_nav2[i]), _t(px_nav2[i + 1]), (255, 0, 0), 2)
        for i in range(len(px_own) - 1):
            cv2.line(img, _t(px_own[i]), _t(px_own[i + 1]), (0, 200, 0), 1)
        cv2.imwrite(args.png, img)
        print('对比图 → %s（蓝=nav2，绿=自研A*）' % args.png)

    return 0 if nav2['verdict'].startswith('✓') else 1


if __name__ == '__main__':
    raise SystemExit(main())
