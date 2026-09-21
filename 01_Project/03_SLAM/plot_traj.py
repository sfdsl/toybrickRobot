#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 goal_nav.py 记录的轨迹 CSV 叠到地图上（N5「记录/归档」用）

链路：
    nav --goal 1.5 2.5 --csv ~/RobotCode/logs/traj_0921_1130.csv     # 跑一次，留下轨迹
    navplot ~/RobotCode/logs/traj_0921_1130.csv --goal 1.5 2.5       # 出图 + 统计

输出：与 CSV 同名的 `<csv>_on_map.png`（灰=未知、白=空闲、黑=障碍、
红线=实际轨迹、绿点=起点、蓝点=终点、红十字=目标点），并打印归档要用的统计值。

用法：
    python3 plot_traj.py <csv> [-o out.png] [--map 前缀] [--goal X Y]
别名：navplot
"""
import argparse
import csv
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import goal_nav as gn          # noqa: E402


def main():
    ap = argparse.ArgumentParser(description='把轨迹 CSV 叠到地图上并统计')
    ap.add_argument('csv', help='goal_nav.py --csv 写出的文件')
    ap.add_argument('-o', '--out', help='输出图片（默认 <csv>_on_map.png）')
    ap.add_argument('--map', default=gn.MAP_DEFAULT, help='地图前缀（默认 %s）' % gn.MAP_DEFAULT)
    ap.add_argument('--goal', nargs=2, type=float, metavar=('X', 'Y'), help='画出目标点')
    a = ap.parse_args()

    rows = []
    with open(a.csv) as f:
        for r in csv.DictReader(f):
            try:
                rows.append([float(r[k]) for k in ('t', 'x', 'y', 'yaw_deg', 'v', 'w', 'd_goal')])
            except (KeyError, ValueError):
                continue
    if len(rows) < 2:
        raise SystemExit('✗ %s 里没有有效轨迹行（列应为 t,x,y,yaw_deg,v,w,d_goal）' % a.csv)

    m = gn.load_map(a.map)
    img = np.full(m['pgm'].shape + (3,), 60, np.uint8)
    img[m['pgm'] == 254] = (235, 235, 235)
    img[m['pgm'] == 0] = (0, 0, 0)
    pts = [gn.world_to_px(m, r[1], r[2]) for r in rows]
    for i in range(len(pts) - 1):
        cv2.line(img, pts[i], pts[i + 1], (0, 0, 255), 1)
    cv2.circle(img, pts[0], 5, (0, 180, 0), -1)            # 起点
    cv2.circle(img, pts[-1], 5, (255, 100, 0), -1)         # 终点
    if a.goal:
        cv2.drawMarker(img, gn.world_to_px(m, a.goal[0], a.goal[1]),
                       (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
    out = a.out or (os.path.splitext(a.csv)[0] + '_on_map.png')
    cv2.imwrite(out, img)

    dur = rows[-1][0] - rows[0][0]
    dist = sum(math.hypot(rows[i + 1][1] - rows[i][1], rows[i + 1][2] - rows[i][2])
               for i in range(len(rows) - 1))
    sp = [r[4] for r in rows]
    print('点数 %d｜时长 %.1f s｜轨迹长 %.2f m｜指令速度 平均 %.3f / 最大 %.3f m/s'
          % (len(rows), dur, dist, sum(sp) / len(sp), max(sp)))
    if rows[-1][6] >= 0:
        print('最终距目标 %.3f m（≤ --tol-xy 即判定到达）' % rows[-1][6])
    print('起点 (%.2f, %.2f) → 终点 (%.2f, %.2f)'
          % (rows[0][1], rows[0][2], rows[-1][1], rows[-1][2]))
    print('图：%s' % out)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
