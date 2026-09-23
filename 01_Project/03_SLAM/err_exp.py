#!/usr/bin/env python3
"""err_exp —— 到达误差量化：目标坐标 + rec_live.log 位姿 → 实际误差。
用法： errexp <目标x> <目标y> [截止秒]
  不带截止秒 → 取文件最后一条位姿（单目标场景）
  带截止秒   → 取 ≤该时刻的最后位姿（多目标长跑按停稳时刻逐段算）
例： errexp 4.56 -0.39 63"""
import math
import re
import sys

if len(sys.argv) < 3:
    print('用法：errexp <目标x> <目标y> [截止秒]')
    sys.exit(1)
gx, gy = float(sys.argv[1]), float(sys.argv[2])
t_cut = float(sys.argv[3]) if len(sys.argv) > 3 else None

path = '/tmp/rec_live.log'
try:
    lines = [l.strip() for l in open(path) if l.strip()]
except FileNotFoundError:
    print('找不到 %s（robotnav 启动的记录器没在跑？）' % path)
    sys.exit(1)

# 行格式：  123s (  4.42,  0.00) yaw=  16.0 v=(0.0, 0.0)
pose = None
for l in reversed(lines):
    m = re.match(r'(\d+)s\s*\(\s*(-?[\d.]+),\s*(-?[\d.]+)\)\s*yaw=\s*(-?[\d.]+)', l)
    if m:
        if t_cut is not None and int(m.group(1)) > t_cut:
            continue          # 截止时刻之后的行跳过（多目标逐段回溯）
        pose = (float(m.group(2)), float(m.group(3)), float(m.group(4)))
        break
if pose is None:
    print('rec_live.log 里没有符合条件的位姿行')
    sys.exit(1)

x, y, yaw = pose
err = math.hypot(x - gx, y - gy)
print('目标   (%.3f, %.3f)' % (gx, gy))
print('终点位姿 (%.3f, %.3f) yaw=%.1f°' % (x, y, yaw))
print('到达误差 = %.3f m' % err)
# P31：三级判定（此前 0.17 m 也显示"✓ 达标"，与 0.15 硬判据矛盾，误导）
if err <= 0.10:
    print('✓ 误差 ≤0.10 m：达标（08 方案切换判据）')
elif err <= 0.15:
    print('⚠ 误差 0.10~0.15 m：达当前 goal tolerance，但超硬判据——检查是否绕障挤压/停偏')
else:
    print('✗ 误差 >0.15 m：超标')
    if err > 0.30:
        print('  且 >0.30 m：大概率定位漂移（map 位姿不准），优先查 slam 定位而非控制')
    elif err > 0.20:
        print('  且 0.20~0.30 m：定位/控制各半嫌疑，做 EKF 前先目视核对 map 位姿与实际位置')
