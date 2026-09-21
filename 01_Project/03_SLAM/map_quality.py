#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""地图质量报告（重新建图前/后的客观对比）—— 离线，只读地图文件，不需要车

三个指标，都是"能不能拿来导航"的直接相关量：
  1. 像素构成：空闲 / 障碍 / 未知 —— 未知区**不可通行**，直接决定可导航范围
  2. 小障碍团簇：椅腿 / 桌腿 / 细杆等**真实障碍**（2026-09-21 实测更正：这些**不是噪点**！）
     —— 默认**保留**（`--min-obstacle-cells 0`）；抹掉 = 规划会直接穿过椅子
  3. 可通行性：某膨胀半径下可通行格占全图比例，以及与**起点连通**的比例 / 最大连通区

本机目标值（2026-09-21 定，重扫后应达标）：
    未知区 ≤ 20%（当前 44.7%）
    膨胀 0.25 m 下可通行 ≥ 25%（不抹任何障碍的口径下统计）

用法：
    python3 map_quality.py                                   # 默认 04_map/robot_map
    python3 map_quality.py --map ~/RobotCode/04_map/map_20260921_1500
    python3 map_quality.py --start 0 0 --png                 # 另存标注图（圈出小障碍）
别名：mapq
"""
import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goal_nav as gn          # noqa: E402

SMALL_CELLS = 4                # "小障碍团簇"的展示口径：<4 格 ≈ 椅腿/桌腿量级


def connectivity(trav, start_px):
    """返回 (可通行格数, 与起点连通格数, 最大连通区格数)"""
    n = int(trav.sum())
    if n == 0:
        return 0, 0, 0
    nlab, lab = cv2.connectedComponents(trav.astype(np.uint8), connectivity=8)
    sizes = [int((lab == i).sum()) for i in range(1, nlab)]
    best = max(sizes) if sizes else 0
    sx, sy = start_px
    conn = 0
    if 0 <= sx < trav.shape[1] and 0 <= sy < trav.shape[0] and trav[sy, sx]:
        conn = int((lab == lab[sy, sx]).sum())
    return n, conn, best


def main():
    ap = argparse.ArgumentParser(description='地图质量报告（重扫前后对比用）')
    ap.add_argument('--map', default=gn.MAP_DEFAULT, help='地图前缀（默认 %s）' % gn.MAP_DEFAULT)
    ap.add_argument('--start', nargs=2, type=float, default=[0.0, 0.0], metavar=('X', 'Y'),
                    help='连通性参照点（默认 0 0 = 建图起点）')
    ap.add_argument('--min-obstacle-cells', type=int, default=0,
                    help='把小于 N 格的障碍当噪点抹掉（**默认 0 = 不抹**；图上小点多为椅腿，'
                         '是真实障碍，抹掉等于规划会穿过去）')
    ap.add_argument('--png', action='store_true', help='另存标注图（红圈圈出小障碍团簇）')
    args = ap.parse_args()

    m = gn.load_map(args.map)
    pgm, w, h, res = m['pgm'], m['w'], m['h'], m['res']
    total = w * h
    free = int((pgm == 254).sum())
    occ = int((pgm == 0).sum())
    unk = total - free - occ
    start_px = gn.world_to_px(m, args.start[0], args.start[1])

    print('地图 %s：%dx%d @%.3f m/px = %.2f m × %.2f m'
          % (os.path.basename(args.map), w, h, res, w * res, h * res))
    print('  像素构成：空闲 %.1f%% ｜ 障碍 %.1f%% ｜ 未知 %.1f%%'
          % (100.0 * free / total, 100.0 * occ / total, 100.0 * unk / total))

    # 小障碍团簇（是真实障碍，不是噪点）
    ocs = (pgm == 0).astype(np.uint8)
    nlab, lab, stats, _ = cv2.connectedComponentsWithStats(ocs, 8)
    small = np.zeros_like(ocs, bool)
    n_small = 0
    for i in range(1, nlab):
        if stats[i, cv2.CC_STAT_AREA] < SMALL_CELLS:
            small |= (lab == i)
            n_small += 1
    n_cells = int(small.sum())
    print('  小障碍团簇：%d 处 / %d 格（<%d 格；多为**椅腿/桌腿等真实障碍**，不是噪点）'
          % (n_small, n_cells, SMALL_CELLS))
    if args.min_obstacle_cells > 0:
        print('    ⚠ 当前 --min-obstacle-cells=%d 会把它们从图上抹掉 → 规划可能直接穿过椅子！'
              % args.min_obstacle_cells)
    else:
        print('    ✓ 默认保留（--min-obstacle-cells 0）：路径会绕开它们（这才是对的）')

    # 不同膨胀半径下的可通行性
    print('  可通行性（未知区按不可通行算；小障碍按 --min-obstacle-cells=%d 处理）：'
          % args.min_obstacle_cells)
    rows = []
    for r in (0.15, 0.20, 0.25):
        trav = gn.build_traversable(m, r, False, args.min_obstacle_cells)
        n, conn, best = connectivity(trav, start_px)
        rows.append((r, 100.0 * n / total, 100.0 * conn / n if n else 0.0,
                     100.0 * best / n if n else 0.0))
        print('    膨胀 %.2f m → 可通行 %5.1f%% ｜ 与起点连通 %5.1f%% ｜ 最大连通区 %5.1f%%'
              % (r, rows[-1][1], rows[-1][2], rows[-1][3]))

    # 判据
    r25 = [x for x in rows if abs(x[0] - 0.25) < 1e-6][0]
    print('  评价（目标：未知 ≤20%%｜膨胀 0.25 m 下可通行 ≥25%%）：')
    bad = []
    if 100.0 * unk / total > 20.0:
        bad.append('未知区 %.1f%% 偏大 → 建议补扫/重扫（灰区不可通行）' % (100.0 * unk / total))
    if r25[1] < 25.0:
        bad.append('膨胀 0.25 m 下可通行仅 %.1f%% → 通道被真实障碍挤窄（椅腿等）或地图不干净'
                   % r25[1])
    if bad:
        for b in bad:
            print('    ✗ %s' % b)
    else:
        print('    ✓ 达标，可以按 0.25 m 膨胀用（更远离墙）')

    if args.png:
        img = np.full((h, w, 3), 60, np.uint8)
        img[pgm == 254] = (235, 235, 235)
        img[pgm == 0] = (0, 0, 0)
        ys, xs = np.nonzero(small)
        for y, x in zip(ys, xs):
            cv2.circle(img, (int(x), int(y)), 6, (0, 0, 255), 1)
        out = os.path.join(os.path.dirname(m['prefix']),
                           os.path.basename(m['prefix']) + '_quality.png')
        cv2.imwrite(out, img)
        print('  标注图（红圈=小障碍团簇，**都是真实障碍**）：%s' % out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
