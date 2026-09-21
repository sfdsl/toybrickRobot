#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""路径离墙距离检查 —— N5「--dry-run 路径不穿墙」的机器判据（离线，不动车）

为什么不是"看图"：图只能看出明显的穿墙，看不出"贴着墙角几厘米掠过"。
本工具把路径**逐像素量成米**，给出可复现的数字与判定（换地图/改参数后可回归）。

复用 goal_nav.py 的规划实现（**不重复实现**，与真机导航同一份代码）：
    gn.GoalNav.plan()    → A* + 目标点吸附（成功/失败原因都一样）
    gn.follow_px_path()  → 抽稀 + 拐角平滑（含越界回退）——控制器 follow() 跟随的就是这条折线
    gn.blocked_mask()    → 与规划器**同口径**的障碍集（含 --min-obstacle-cells）

三个口径（这是关键——判据必须和规划器一致，否则量的是它有意绕开的东西）：
    ① 真实障碍（含椅腿/桌腿等小障碍，未膨胀）：物理安全 —— 车会不会蹭上
       （判据 ≥ --min-clear，默认 0.12 m = 车半宽。**注意小点是真实障碍，不是噪点**）
    ② 不可通行区（障碍 ∪ 未知，膨胀前）：规划器口径 —— 有没有出可通行区（判据 ≥ --inflation）
    ③ 被忽略的小障碍（仅 --min-obstacle-cells>0 时才出现）：信息行，
       提示"规划可能从椅腿等真实障碍上直接穿过去"

判定：
    ✗ 穿墙        路径压到真实障碍像素
    ✗ 过紧        离真实障碍 < --min-clear
    ⚠ 出可通行区  离不可通行区 < --inflation（A* 链不会，抽稀/平滑可能切角造成）
    ✓ 通过        以上都不成立

用法（需先 source ROS 环境：本脚本 import goal_nav，它会 import rclpy）：
    python3 check_path.py --goal 2.5 -1.0                      # 单目标（起点默认 0,0）
    python3 check_path.py --goal 2.0 1.5 --start 0.5 -0.2      # 指定起点
    python3 check_path.py --goals "2.5,-1.0; -1.5,-3.5; 2.0,1.5"   # 批量
别名：navcheck
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goal_nav as gn          # noqa: E402


class _Log:
    """GoalNav.plan() 会调 self.get_logger().warn() —— 给个最小替身"""
    def warn(self, s):
        print('    ! %s' % s)

    def info(self, s):
        print('    %s' % s)


class _Shim:
    """只为复用 GoalNav.plan()：该方法用到 m / trav / blocked / cost / pose / get_logger()"""
    def __init__(self, m, trav, pose, blocked=None, cost=None):
        self.m, self.trav, self.pose = m, trav, pose
        self.blocked, self.cost = blocked, cost     # blocked 未膨胀（"走宽处"代价用）

    def get_logger(self):
        return _Log()


def dist_map(mask, res):
    """True=障碍的掩码 → 每格到最近障碍的距离（米）；掩码全空返回 None"""
    if not mask.any():
        return None
    return cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 5) * res


def rasterize(px_path, w, h):
    """折线 → 像素点列表 [(col,row), ...]（逐段直线栅格化，含端点；点可为小数，四舍五入）"""
    ip = [(int(round(p[0])), int(round(p[1]))) for p in px_path]
    mask = np.zeros((h, w), np.uint8)
    for i in range(len(ip) - 1):
        cv2.line(mask, ip[i], ip[i + 1], 1, 1)
    rr, cc = np.nonzero(mask)
    return list(zip(cc.tolist(), rr.tolist()))


def min_on_path(pts, d):
    """路径上到该障碍集的最近距离与该点像素（d 为 None 时返回 (inf, pts[0])）"""
    if d is None:
        return float('inf'), pts[0]
    best, bpx = 1e9, pts[0]
    for (c, r) in pts:
        v = float(d[r, c])
        if v < best:
            best, bpx = v, (c, r)
    return best, bpx


def verdict(min_obs, min_blk, inflation, min_clear):
    if min_obs <= 1e-6:
        return '✗ 穿墙', '路径压到真实障碍像素'
    if min_obs < min_clear:
        return '✗ 过紧', '离真实障碍 %.3f m < 车半宽 %.2f m' % (min_obs, min_clear)
    if min_blk < inflation - 1e-6:
        return '⚠ 出可通行区', '离不可通行区 %.3f m < 膨胀半径 %.2f m' % (min_blk, inflation)
    return '✓ 通过', ''


def clearance_stats(px_path, dist_map, w, h):
    """沿路径的离障碍距离统计：(最小, 平均, 窄处占比)

    "窄处" = 离障碍 < 0.35 m 的栅格（椅腿缝那个量级）。只看"最小距离"会被
    终点附近（目标点常贴着椅子）拖低，看不出整条路径走的是不是敞亮地方，
    所以这里同时给平均距离和窄处占比。
    """
    vals = [float(dist_map[r, c]) for c, r in rasterize(px_path, w, h)]
    if not vals:
        return 0.0, 0.0, 0.0
    return (min(vals), sum(vals) / len(vals),
            sum(1 for v in vals if v < 0.35) / float(len(vals)))


def main():
    ap = argparse.ArgumentParser(description='路径离墙距离检查（N5 第 2 项判据）')
    ap.add_argument('--map', default=gn.MAP_DEFAULT, help='地图前缀（默认 %s）' % gn.MAP_DEFAULT)
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='起点地图坐标（默认 0 0 = 建图起点）')
    ap.add_argument('--goal', nargs='+', type=float, metavar='X Y', help='单个目标点')
    ap.add_argument('--goals', help='批量目标："x,y; x,y; ..."')
    ap.add_argument('--inflation', type=float, default=0.15, help='膨胀半径 m（默认同 goal_nav）')
    ap.add_argument('--min-clear', type=float, default=0.12,
                    help='离真实障碍的最小允许距离 m（默认 0.12 = 车半宽）')
    ap.add_argument('--min-obstacle-cells', type=int, default=0,
                    help='把小于 N 格的障碍团簇抹掉（**默认 0 = 不抹**，与 goal_nav 一致）。'
                         '⚠ 图上小点多为椅腿等真实障碍，抹掉＝规划会穿过去（2026-09-21 更正）')
    ap.add_argument('--allow-unknown', action='store_true', help='允许穿过未知区（默认禁止）')
    ap.add_argument('--smooth-iter', type=int, default=2,
                    help='拐角平滑迭代次数（默认 2，与 goal_nav 一致；0=关闭，便于对比）')
    ap.add_argument('--prefer-open', type=float, default=3.0,
                    help='"走宽处"偏好强度（默认 3，与 goal_nav 一致；0=关 → 纯最短路径，便于对比）')
    ap.add_argument('--open-dist', type=float, default=0.8,
                    help='多宽算"宽敞" m（默认 0.8，与 goal_nav 一致）')
    args = ap.parse_args()

    goals = []
    if args.goal:
        goals.append((args.goal[0], args.goal[1]))
    if args.goals:
        for seg in args.goals.split(';'):
            seg = seg.strip()
            if seg:
                x, y = [float(v) for v in seg.replace(',', ' ').split()]
                goals.append((x, y))
    if not goals:
        print('✗ 需要 --goal x y 或 --goals "x,y; x,y"')
        return 2

    m = gn.load_map(args.map)
    trav = gn.build_traversable(m, args.inflation, args.allow_unknown, args.min_obstacle_cells)
    blocked, removed = gn.blocked_mask(m, args.allow_unknown, args.min_obstacle_cells)
    clean_obs = blocked & (m['pgm'] == 0)          # 真实障碍：去噪后、未膨胀
    noise = removed & (m['pgm'] == 0)              # 被当噪点忽略的小团簇（信息行）
    d_obs = dist_map(clean_obs, m['res'])
    d_blk = dist_map(blocked, m['res'])
    d_noise = dist_map(noise, m['res'])
    n_noise = cv2.connectedComponentsWithStats(noise.astype(np.uint8), 8)[0] - 1 if noise.any() else 0

    cost = gn.clearance_cost(m, blocked, args.open_dist, args.prefer_open)
    shim = _Shim(m, trav, (args.start[0], args.start[1], 0.0), blocked, cost)

    print('地图 %s：%dx%d @%.3f m/px｜膨胀 %.2f m｜小障碍阈值 %d｜判据：真实障碍 ≥%.2f m%s'
          % (os.path.basename(args.map), m['w'], m['h'], m['res'], args.inflation,
             args.min_obstacle_cells, args.min_clear,
             '｜允许未知区' if args.allow_unknown else ''))
    if args.min_obstacle_cells > 0:
        print('⚠ 抹掉 %d 处小障碍（共 %d 格）——图上小点多为椅腿，是**真实障碍**！'
              % (n_noise, int(noise.sum())))
    else:
        print('小障碍：全部保留（--min-obstacle-cells 0，与 goal_nav 默认一致）')
    print('走宽处偏好：prefer-open=%.1f（open-dist %.1f m；0=纯最短路径）\n'
          % (args.prefer_open, args.open_dist))
    print('起点 (%.2f, %.2f)　目标 %d 个\n' % (args.start[0], args.start[1], len(goals)))

    rows, ok_n = [], 0
    for i, (gx, gy) in enumerate(goals, 1):
        print('=== [%d] 目标 (%.2f, %.2f) ===' % (i, gx, gy))
        path, why = gn.GoalNav.plan(shim, (gx, gy))
        if path is None:
            print('  规划失败：%s\n' % why)
            rows.append(('(%.2f, %.2f)' % (gx, gy), '—', '规划失败：%s' % why, '✗'))
            continue

        simp, why_post = gn.follow_px_path(m, trav, path, 0.1, args.smooth_iter)
        in_obs, px_obs = min_on_path(rasterize(simp, m['w'], m['h']), d_obs)
        in_blk, px_blk = min_on_path(rasterize(simp, m['w'], m['h']), d_blk)
        raw_obs, _ = min_on_path(rasterize(path, m['w'], m['h']), d_obs)   # A* 原始链参考
        _, avg_obs, narrow = clearance_stats(simp, d_obs, m['w'], m['h'])
        print('  ④ 走宽处：平均离障碍 %.3f m｜窄处(<0.35 m)占比 %.0f%%'
              % (avg_obs, 100.0 * narrow))
        mark, detail = verdict(in_obs, in_blk, args.inflation, args.min_clear)
        n_obs_dist, px_noise = min_on_path(rasterize(simp, m['w'], m['h']), d_noise)

        print('  规划成功（%s）：A* 路径 %.2f m / %d 点 → 抽稀+平滑 %d 航点%s'
              % (why, gn.path_length_m(m, path), len(path), len(simp),
                 '（%s）' % why_post if why_post else ''))
        print('  ① 真实障碍（含椅腿等小障碍）：最近 %.3f m @ 世界(%.2f, %.2f)（判据 ≥%.2f %s）'
              % (in_obs, *gn.px_to_world(m, *px_obs), args.min_clear,
                 '✓' if in_obs >= args.min_clear else '✗'))
        print('  ② 不可通行区：最近 %.3f m（膨胀 %.2f %s）｜A* 原始链 %.3f m'
              % (in_blk, args.inflation, '✓' if in_blk >= args.inflation - 1e-6 else '⚠', raw_obs))
        if n_obs_dist < args.inflation:
            print('  ③ 被忽略的小障碍：最近 %.3f m @ 世界(%.2f, %.2f) ← 多为椅腿，是真实障碍！'
                  % (n_obs_dist, *gn.px_to_world(m, *px_noise)))
        print('  判定：%s %s' % (mark, detail))

        out = os.path.join(os.path.dirname(m['prefix']),
                           os.path.basename(m['prefix']) + '_check%s.png'
                           % ('' if len(goals) == 1 else str(i)))
        gn.draw_overlay(m, trav, path, gn.world_to_px(m, *args.start), gn.world_to_px(m, gx, gy), out)
        img = cv2.imread(out)
        ip_simp = [(int(round(p[0])), int(round(p[1]))) for p in simp]
        for j in range(len(ip_simp) - 1):                  # 蓝线 = 控制器实际跟随的折线
            cv2.line(img, ip_simp[j], ip_simp[j + 1], (255, 0, 0), 1)
        cv2.circle(img, px_obs, 6, (255, 0, 255), 1)       # 品红 = 离真实障碍最近处
        cv2.putText(img, '%.3fm' % in_obs, (px_obs[0] + 8, px_obs[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)
        if n_obs_dist < args.inflation:                    # 灰圈 = 被忽略的噪点
            cv2.circle(img, px_noise, 6, (150, 150, 150), 1)
        cv2.imwrite(out, img)
        print('  图：%s\n' % out)

        if mark.startswith('✓'):
            ok_n += 1
        rows.append(('(%.2f, %.2f)' % (gx, gy), '%.2f m' % gn.path_length_m(m, path),
                     '真实障碍 %.3f m｜不可通行区 %.3f m' % (in_obs, in_blk), mark))

    print('=== 汇总：%d/%d 通过 ===' % (ok_n, len(rows)))
    for r in rows:
        print('  %-16s %-8s %-44s %s' % r)
    return 0 if ok_n == len(rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
