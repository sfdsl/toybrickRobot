#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""目标点自主导航：已知地图 → A* 规划 → 纯跟踪控制 → 开过去

流程：读地图（PGM+YAML）→ 障碍膨胀 → A* 全局路径 → 纯跟踪发 /cmd_vel → 到点停车
前提：slam_toolbox 跑【定位模式】：
      ros2 launch slam_toolbox localization_launch.py     # 加载 04_map/robot_map 做定位
      （TF 链：map→odom→base_link→base_laser）

用法：
    python3 goal_nav.py --goal 2.0 1.5             # 去地图坐标 (2.0, 1.5)
    python3 goal_nav.py --goal 2.0 1.5 90          # 到点后车头转到 90°（地图坐标系，可选）
    python3 goal_nav.py --dry-run --start 0 0 --goal 2 1.5   # 只规划、出路径图，不动车
    python3 goal_nav.py --listen                   # 等 /goal_pose 话题（手机/其他程序下发）
参数：
    --map <前缀>        地图文件前缀（默认 ~/RobotCode/04_map/robot_map）
    --inflation 0.25    障碍膨胀半径 m（车半径 0.12 + 余量）
    --allow-unknown     允许穿过未知区域（默认**禁止**：灰区不可通行）
    --vmax 0.15         直行速度上限 m/s（雷达只有 6Hz，建议 ≤0.2）
    --wmax 0.8          转向速度上限 rad/s
    --lookahead 0.4     纯跟踪前视距离 m
    --tol-xy 0.15       到点判定（米）；--tol-yaw 10（度）
    --topic /cmd_vel    速度话题（自测可换测试话题，不动真车）

安全（沿用 auto_drive.py 的三层思路）：
    * /tf（map→base_link）或 /odom 断流 >1.5s → 立即停车退出
    * /scan 前向 ±30° 扇区：<0.25m 减速；<0.15m 急停等待（>3s 触发重新规划）
    * 规划偏好"走宽处"：A* 代价含"离障碍越近代价越高"项（椅腿缝里钻很贵）→ --prefer-open
    * 路径偏离 >0.6m 或前方长时间被堵 → 重新规划（最多 5 次）
    * 总超时保护；Ctrl-C 立即停车；**不发横移速度 vy**
"""
import argparse
import heapq
import math
import os
import re
import time

import numpy as np

try:
    import cv2
    import rclpy
    import rclpy.time
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener
except ImportError as e:
    raise SystemExit(
        '导入模块失败：%s\n提示：先 source 环境\n'
        '  source /opt/ros/humble/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

MAP_DEFAULT = os.path.expanduser('~/RobotCode/04_map/robot_map')
TF_TIMEOUT = 1.5        # 位姿断流判据（秒）
# ⚠ 2026-09-22 实测事故后上调：原 0.15 / 0.25 是"贴脸才停"的口径——
#   人站在车前约 1.5 m 处（雷达确实读到 1.497 m），车照样直冲，到 0.15 m 才急停，
#   等于怼到人身上才停。安全层是给人挡的，不是给纸箱挡的，保护区必须到 1 m 量级。
STOP_NEAR = 0.45        # 前向急停距离 m（原 0.15）
SLOW_NEAR = 1.00        # 前向减速距离 m（原 0.25）
SECTOR_DEG = 30.0       # 前向扇区半角


# ==========================================================================
#  地图与规划（纯函数，可离线测试）
# ==========================================================================
def load_map(prefix):
    """读 PGM + YAML → dict(pgm, res, ox, oy, w, h)

    `prefix` 是**不带扩展名**的前缀；顺手容忍误带扩展名（.pgm/.yaml/.posegraph/.data，
    省得每次都要记"到底带不带 .pgm"）。
    PGM 由 save_map.py 生成：白 254=空闲、黑 0=障碍、灰 205=未知；
    图像第 0 行对应地图的【最大 y】（map_server 约定）。
    """
    for ext in ('.posegraph', '.data', '.yaml', '.yml', '.pgm'):
        if prefix.endswith(ext):
            prefix = prefix[:-len(ext)]
            break
    pgm_path, yml_path = prefix + '.pgm', prefix + '.yaml'
    with open(pgm_path, 'rb') as f:
        data = f.read()
    if not data.startswith(b'P5'):
        raise SystemExit('%s 不是 P5 二进制 PGM' % pgm_path)
    # 头：P5 / 宽 高 / 最大值（可能夹注释）
    tokens, i = [], 2
    while len(tokens) < 3:
        while i < len(data) and data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            while i < len(data) and data[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(data) and not data[j:j + 1].isspace():
            j += 1
        tokens.append(int(data[i:j]))
        i = j
    w, h = tokens[0], tokens[1]
    pgm = np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8).reshape(h, w)

    res = 0.05
    ox = oy = 0.0
    with open(yml_path) as f:
        for line in f:
            m = re.match(r'\s*resolution:\s*([-\d.eE]+)', line)
            if m:
                res = float(m.group(1))
            m = re.match(r'\s*origin:\s*\[([^\]]+)\]', line)
            if m:
                vals = [float(v) for v in m.group(1).split(',')]
                ox, oy = vals[0], vals[1]
    return {'pgm': pgm, 'res': res, 'ox': ox, 'oy': oy, 'w': w, 'h': h,
            'prefix': prefix}


def world_to_px(m, x, y):
    """世界坐标(米) → 像素 (col, row)，row 从上往下"""
    col = int(round((x - m['ox']) / m['res']))
    row = int(round((m['oy'] + m['h'] * m['res'] - y) / m['res']))
    return col, row


def px_to_world(m, col, row):
    x = m['ox'] + (col + 0.5) * m['res']
    y = m['oy'] + (m['h'] - row - 0.5) * m['res']
    return x, y


def blocked_mask(m, allow_unknown=False, min_obstacle_cells=0):
    """不可通行掩码（**膨胀前**）：障碍；未 allow_unknown 时连未知一起算

    min_obstacle_cells > 0：把小于该格数的团簇抹掉，被抹掉的格子由 `removed` 返回
    （check_path.py 用它区分"保留的障碍"与"被忽略的点"）。
    ⚠ 2026-09-21 实测更正：地图上的小点多是**椅腿 / 桌腿**等**真实障碍**，不是噪点；
    默认 **0 = 不抹**。调大它路径会"变短变直"，但那只是把真实障碍从图上删掉的结果
    —— 车可能直接撞上去（除非能确认是残差，否则不要动）。

    返回 (blocked, removed)。
    """
    blocked = m['pgm'] == 0
    if not allow_unknown:
        blocked = blocked | (m['pgm'] != 254)          # 未知也算障碍
    removed = np.zeros_like(blocked)
    if min_obstacle_cells > 0:
        n, lab, stats, _ = cv2.connectedComponentsWithStats(blocked.astype(np.uint8), 8)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] < min_obstacle_cells:
                removed |= (lab == i)
        blocked = blocked & ~removed
    return blocked, removed


def build_traversable(m, inflation=0.15, allow_unknown=False, min_obstacle_cells=0):
    """返回可通行布尔图：空闲格 + 距障碍/未知 ≥ inflation（噪点清理见 blocked_mask）"""
    blocked, _ = blocked_mask(m, allow_unknown, min_obstacle_cells)
    r = int(round(inflation / m['res']))
    if r > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
        blocked = cv2.dilate(blocked.astype(np.uint8), k).astype(bool)
    return (m['pgm'] == 254) & ~blocked


def dist_to_obstacles(m, blocked):
    """每格到最近障碍的距离（米）——"走宽处"代价与"路径自检"共用同一张图"""
    return cv2.distanceTransform((~blocked).astype(np.uint8) * 255,
                                 cv2.DIST_L2, 5) * m['res']


def clearance_cost(m, blocked, open_dist=0.8, prefer_open=3.0, tight=0.35):
    """给"贴着障碍走"加价 → A* 宁愿绕远走敞亮处（**软约束，不改变可达性**）

    对**未膨胀**的障碍图（含椅腿等小障碍、未知区）做距离变换（单位米）：
    离障碍 d 米的格子，每步代价乘以

        1 + prefer_open * ((open_dist − d) / open_dist)²      （d ≥ open_dist 时 = 1，不加价）

    离障碍 < tight（默认 0.35 m ≈ 椅腿缝那个量级）再额外 ×3 —— 逼它别从缝里钻。
    `prefer_open = 0` → 返回 None（纯最短路径，即旧行为）。

    与 `--inflation` 的分工：inflation 是**硬约束**（永远不许比它更近），
    这里是**软偏好**（不禁止，但很贵），所以不会像调大 inflation 那样把可达区吃掉。
    """
    if prefer_open <= 0:
        return None
    d = dist_to_obstacles(m, blocked)
    over = np.clip((open_dist - d) / max(1e-6, open_dist), 0.0, 1.0)
    cost = 1.0 + prefer_open * over ** 2
    cost[d < tight] *= 3.0
    return cost.astype(np.float32)


def path_clearance_stats(m, d_obs, px_path):
    """沿折线的"离真实障碍距离"统计：(最小, 平均, 窄处占比)

    px_path = 像素折线（A* 原始链或抽稀+平滑后都行）。按 1 px 步长在每段上插值取距离，
    量的是**车实际经过的那条线**，而不是只看拐点。
    "窄处"= 离障碍 < 0.35 m（椅腿缝量级），与 `navcheck` 的 ④ 行同口径。
    """
    vals = []
    if px_path:
        # 注意：平滑后的折线是**浮点**坐标（Chaikin 取中点），索引前必须取整
        x0, y0 = int(round(px_path[0][0])), int(round(px_path[0][1]))
        if 0 <= x0 < m['w'] and 0 <= y0 < m['h']:
            vals.append(float(d_obs[y0, x0]))
    for (x0, y0), (x1, y1) in zip(px_path, px_path[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0)))
        for k in range(n + 1):
            t = k / n
            x = int(round(x0 + (x1 - x0) * t))
            y = int(round(y0 + (y1 - y0) * t))
            if 0 <= x < m['w'] and 0 <= y < m['h']:
                vals.append(float(d_obs[y, x]))
    if not vals:
        return 0.0, 0.0, 0.0
    return (min(vals), sum(vals) / len(vals),
            sum(1 for v in vals if v < 0.35) / float(len(vals)))


def astar(trav, start, goal, cost=None):
    """8 邻域 A*（对角代价 √2，octile 启发）；返回像素点列表或 None

    cost: 可选的"每格代价系数"数组（≥1，见 clearance_cost()）——取相邻两格系数的
          平均值乘到步长上。给 None 就是纯最短路（旧行为）。"""

    h, w = trav.shape
    if not trav[goal[1], goal[0]]:
        return None
    def ok(p):
        return 0 <= p[0] < w and 0 <= p[1] < h and trav[p[1], p[0]]
    if not ok(start):
        return None
    nbr = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
           (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
           (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))]
    openq = [(0.0, start)]
    came, g = {}, {start: 0.0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for dx, dy, c in nbr:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not ok((nx, ny)):
                continue
            if dx and dy:                      # 斜向不切角
                if not (trav[cur[1]][nx] and trav[ny][cur[0]]):
                    continue
            step = c if cost is None else c * 0.5 * (cost[cur[1], cur[0]] + cost[ny, nx])
            ng = g[cur] + step
            n = (nx, ny)
            if ng < g.get(n, 1e18):
                g[n] = ng
                came[n] = cur
                f = ng + math.hypot(nx - goal[0], ny - goal[1]) \
                    + (math.sqrt(2) - 2) * min(abs(nx - goal[0]), abs(ny - goal[1]))
                heapq.heappush(openq, (f, n))
    return None


def simplify_path(path, min_step=3):
    """抽稀：去掉共线的中间点（保留拐点）；min_step 控制最小间隔（像素）"""
    if len(path) <= 2:
        return path
    out = [path[0]]
    for i in range(1, len(path) - 1):
        a, b, c = out[-1], path[i], path[i + 1]
        if (b[0] - a[0]) * (c[1] - a[1]) != (c[0] - a[0]) * (b[1] - a[1]):
            if math.hypot(b[0] - out[-1][0], b[1] - out[-1][1]) >= min_step:
                out.append(b)
    out.append(path[-1])
    return out


def path_length_m(m, path):
    return sum(math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
               for i in range(len(path) - 1)) * m['res']


def smooth_path(px_path, m, trav, iterations=2):
    """拐角平滑（Chaikin 切角，**逐拐角验收**）+ 整条复查；返回 (折线, 说明)

    每个拐角单独试切：切出的短边若越出可通行区（trav）就**只把这个拐角保持原样**
    （其余拐角照常圆化，而不是整条一起放弃）。最后对整条折线再做一次逐格复查——
    任何越界一律回退成平滑前的折线（安全优先）。
    说明串为空表示没动（迭代 0 或点太少）。
    """
    if iterations <= 0 or len(px_path) < 3:
        return list(px_path), ''
    h, w = trav.shape

    def seg_ok(p, q):
        """线段 p→q（允许小数）是否全程在可通行区内"""
        mask = np.zeros((h, w), np.uint8)
        cv2.line(mask, (int(round(p[0])), int(round(p[1]))),
                 (int(round(q[0])), int(round(q[1]))), 1, 1)
        return not np.any((mask == 1) & (~trav))

    pts = [(float(p[0]), float(p[1])) for p in px_path]
    cut, tries = 0, 0
    for _ in range(iterations):
        out = [pts[0]]
        for i in range(1, len(pts) - 1):
            a, b, c = out[-1], pts[i], pts[i + 1]
            q = (0.75 * b[0] + 0.25 * a[0], 0.75 * b[1] + 0.25 * a[1])
            r = (0.75 * b[0] + 0.25 * c[0], 0.75 * b[1] + 0.25 * c[1])
            tries += 1
            if seg_ok(q, r):            # 只查新切出的那条短边（另两段是原线段的子段，必然安全）
                out.extend([q, r])
                cut += 1
            else:
                out.append(b)           # 这个拐角太紧 → 保持原样
        out.append(pts[-1])
        pts = out
    ip = [(int(round(x)), int(round(y))) for x, y in pts]
    mask = np.zeros((h, w), np.uint8)
    for i in range(len(ip) - 1):
        cv2.line(mask, ip[i], ip[i + 1], 1, 1)
    bad = int(np.count_nonzero((mask == 1) & (~trav)))
    if bad:
        return list(px_path), '平滑后整条复查有 %d 格越界 → 已回退原折线' % bad
    if cut == 0:
        return list(px_path), '拐角都太紧（%d 次试切均越界）→ 保持原折线' % tries
    return pts, '平滑 %d/%d 次拐角切角（%d → %d 点）' % (cut, tries, len(px_path), len(pts))


def follow_px_path(m, trav, path, step_m=0.1, smooth_iter=2):
    """控制器实际跟随的折线：抽稀（~step_m 一个航点）→ 拐角平滑（含越界回退）

    follow() 与 check_path.py **共用这一条链路**，保证"navcheck 检查的"就是"车实际走的"。
    返回 (折线, 说明)。
    """
    step = max(1, int(step_m / m['res']))
    return smooth_path(simplify_path(path, step), m, trav, smooth_iter)


def snap_goal_px(m, trav, g_px, radius_m=0.5):
    """目标点净化：目标像素若不可通行，就近吸附到可通行格（半径 radius_m 内）

    **自研 `GoalNav.plan()` 与 `nav2_goal_bridge` 共用**（同一个"目标点净化"口径）——
    nav2 自带的终点吸附只保证"不压到致命格"，会停在贴墙 0.11 m 处（实测，< 车半宽 0.12）；
    自研的吸附基于**膨胀后**的可通行区，吸附点必然离障碍 ≥ 膨胀半径，车真能停进去。
    所以换 nav2 后仍要用这个函数先净化目标点，再下发给 nav2。

    返回 (g_px, snapped_world)：
      · 目标本来就可通行 → (原 g_px, None)
      · 吸附成功         → (吸附后像素, (x, y))
      · 找不到可通行格   → (None, None)
    """
    if trav[g_px[1], g_px[0]]:
        return g_px, None
    r = int(round(radius_m / m['res']))
    best = None
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            c, rw = g_px[0] + dx, g_px[1] + dy
            if 0 <= c < m['w'] and 0 <= rw < m['h'] and trav[rw, c]:
                d = math.hypot(dx, dy)
                if best is None or d < best[0]:
                    best = (d, (c, rw))
    if best is None:
        return None, None
    return best[1], px_to_world(m, *best[1])


def draw_overlay(m, trav, path, start_px, goal_px, out_png):
    """把可通行区/路径/起终点画成 PNG（--dry-run 用）"""
    img = np.full(m['pgm'].shape + (3,), 60, np.uint8)
    img[m['pgm'] == 254] = (235, 235, 235)
    img[m['pgm'] == 0] = (0, 0, 0)
    img[trav] = (200, 235, 200)                      # 可通行区淡绿
    for i in range(len(path) - 1):
        cv2.line(img, path[i], path[i + 1], (0, 0, 255), 2)
    cv2.circle(img, start_px, 5, (0, 180, 0), -1)     # 起点 绿
    cv2.circle(img, goal_px, 5, (255, 100, 0), -1)    # 终点 蓝
    cv2.imwrite(out_png, img)
    return out_png


# ==========================================================================
#  导航节点
# ==========================================================================
class GoalNav(Node):
    def __init__(self, args):
        super().__init__('goal_nav')
        self.a = args
        self.m = load_map(args.map)
        self.trav = build_traversable(self.m, args.inflation, args.allow_unknown,
                                      args.min_obstacle_cells)
        # "走宽处"（软偏好）：用**未膨胀**的障碍图（含椅腿/未知区）算离障碍距离 → A* 逐格代价
        self.blocked, _ = blocked_mask(self.m, args.allow_unknown, args.min_obstacle_cells)
        self.cost = clearance_cost(self.m, self.blocked, args.open_dist, args.prefer_open)
        self.d_obs = dist_to_obstacles(self.m, self.blocked)   # 路径自检用（每格离障碍多远）
        self.vmax = args.vmax
        self.wmax = args.wmax
        self.pose = None            # (x, y, yaw) 地图系
        self.last_pose_t = 0.0
        self.scan_min_front = None  # 前向最近距离
        self.active = 0.0           # 最近一次收到速度指令的时间
        self.goal = None            # (x, y, yaw or None)
        self.goal_seq = 0           # 每收到一次 /goal_pose 递增（用于"途中换目标"检测）
        self.t_start = time.time()
        self.last_d_goal = None     # 最近一次算出的"距终点"（写 CSV 用）
        self.csv_f, self.last_csv_t = None, 0.0
        if args.csv:                # 轨迹文件（N5 归档：可直接叠到地图上画）
            self.csv_f = open(args.csv, 'w')
            self.csv_f.write('t,x,y,yaw_deg,v,w,d_goal\n')

        self.pub = self.create_publisher(Twist, args.topic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan,
                                 qos_profile_sensor_data)
        if args.listen:
            self.create_subscription(PoseStamped, args.goal_topic, self.on_goal, 10)
            self.get_logger().info('等待 %s ...（frame_id 应为 map）' % args.goal_topic)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    # ---------- 回调 ----------
    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_scan(self, msg):
        n = len(msg.ranges)
        if n == 0:
            return
        # 前向 ±SECTOR_DEG 扇区：先找到"角度 0°(正前方)"对应的索引，再左右各取 half 个
        # （不假设索引 0 就是正前方——LD14P 的角度范围是 -π~+π，正前方在中间）
        res_ang = (msg.angle_max - msg.angle_min) / max(1, n - 1)
        i0 = int(round((0.0 - msg.angle_min) / max(1e-9, res_ang))) % n
        half = max(1, int(round(math.radians(self.a.sector_deg) / max(1e-9, abs(res_ang)))))
        idx = [(i0 + k) % n for k in range(-half, half + 1)]
        vals = [msg.ranges[i] for i in idx
                if msg.ranges[i] == msg.ranges[i] and msg.ranges[i] > 0.02]
        self.scan_min_front = min(vals) if vals else None

    def on_goal(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        q = msg.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        # 同一个点的**重复下发**（网页/脚本可能按固定频率重发）不算新目标，
        # 否则车会不停"放弃当前路径→改道"，永远走不到
        if self.goal is not None:
            same_xy = abs(self.goal[0] - x) < 0.10 and abs(self.goal[1] - y) < 0.10
            same_yaw = abs((self.goal[2] or 0.0) - yaw) < math.radians(10)
            if same_xy and same_yaw:
                return
        self.get_logger().info('收到目标点 (%.2f, %.2f) yaw=%.0f°' % (x, y, math.degrees(yaw)))
        self.goal = (x, y, yaw)
        self.goal_seq += 1      # 标记"新目标"：行驶中被 follow() 检测到 → 放弃原路径改道

    # ---------- 基础 ----------
    def spin_once(self, timeout=0.05):
        rclpy.spin_once(self, timeout_sec=timeout)

    def send(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)
        self.last_cmd_t = time.time()
        self.log_csv(float(vx), float(wz))

    def log_csv(self, v, w):
        """轨迹记录（≤5 Hz 一行）：t, x, y, yaw, v, w, 距终点；--csv 未给则不写"""
        if self.csv_f is None or self.pose is None:
            return
        t = time.time()
        if t - self.last_csv_t < 0.2:
            return
        self.last_csv_t = t
        self.csv_f.write('%.2f,%.3f,%.3f,%.1f,%.3f,%.3f,%.3f\n'
                         % (t - self.t_start, self.pose[0], self.pose[1],
                            math.degrees(self.pose[2]), v, w,
                            self.last_d_goal if self.last_d_goal is not None else -1.0))
        self.csv_f.flush()

    def stop(self, secs=0.4):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < secs:
            self.send(0.0, 0.0)
            self.spin_once()

    def update_pose(self):
        """从 TF 取位姿（默认 map→base_link，可用 --tf-parent/--tf-child 换），更新 self.pose"""
        try:
            tr = self.tf.lookup_transform(self.a.tf_parent, self.a.tf_child,
                                          rclpy.time.Time())
            t, q = tr.transform.translation, tr.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
            self.pose = (t.x, t.y, yaw)
            self.last_pose_t = time.time()
        except Exception:
            pass
        return (time.time() - self.last_pose_t) < TF_TIMEOUT

    def check_alive(self):
        if time.time() - self.last_pose_t > TF_TIMEOUT:
            raise RuntimeError('位姿断流超过 %.1fs（定位/SLAM 停了？）' % TF_TIMEOUT)

    # ---------- 规划 ----------
    def plan(self, goal_xy=None):
        """从当前位姿规划到目标；返回 (path_px, 说明) 或 (None, 原因)"""
        gx, gy = goal_xy if goal_xy else self.goal[:2]
        s_px = world_to_px(self.m, self.pose[0], self.pose[1])
        g_px = world_to_px(self.m, gx, gy)
        snap = None
        if not (0 <= g_px[0] < self.m['w'] and 0 <= g_px[1] < self.m['h']):
            return None, '目标点在地图范围外'
        # 目标点净化（与 nav2_goal_bridge 共用的同一个函数）
        g_px, snap = snap_goal_px(self.m, self.trav, g_px)
        if g_px is None:
            return None, '目标点落在障碍/未知区（0.5m 内无可通行格）'
        if snap:
            self.get_logger().warn('目标点不可通行 → 就近吸附到 (%.2f, %.2f)' % snap)
        if not self.trav[s_px[1], s_px[0]]:
            return None, '当前位姿不在可通行区（车贴障碍太近？把车挪 20cm 重试）'
        path = astar(self.trav, s_px, g_px, self.cost)   # self.cost=None → 纯最短路
        if path is None:
            return None, 'A* 找不到路径（目标被障碍隔开？试试 --inflation 小一点）'
        return path, ('吸附到可通行点' if snap else 'ok')

    # ---------- 控制（纯跟踪） ----------
    def follow(self, path):
        """沿路径开；返回 'arrived' / 'replan'"""
        px_path, why_post = follow_px_path(self.m, self.trav, path,
                                           smooth_iter=self.a.smooth_iter)
        if why_post:
            self.get_logger().info('路径后处理：%s' % why_post)
        # 路径自检（对应 navcheck 的 ①④ 行）：量一遍车实际要走的那条线离障碍多远
        mn, avg, narrow = path_clearance_stats(self.m, self.d_obs, px_path)
        self.get_logger().info('路径自检：离真实障碍最近 %.3f m｜平均 %.3f m｜窄处(<0.35 m) %.0f%%'
                               % (mn, avg, 100.0 * narrow))
        if mn < 0.12:
            self.get_logger().warn('⚠ 路径最近只有 %.3f m（< 车半宽 0.12 m）→ 建议换目标点；'
                                   '确需通过就先把挡路的椅子挪开或重扫地图' % mn)
        wp = [px_to_world(self.m, *p) for p in px_path]
        total = path_length_m(self.m, px_path)
        my_seq = self.goal_seq             # 记下出发时的目标序号（途中换目标要能看出来）
        t_start = time.time()
        timeout = total / max(0.05, self.vmax) * 3.0 + 20.0
        block_t0 = None
        self.get_logger().info('开始导航：%d 个航点，路径长 %.2f m，限时 %.0fs'
                               % (len(wp), total, timeout))

        while True:
            self.spin_once()
            self.check_alive()
            if time.time() - t_start > timeout:
                raise RuntimeError('导航超时（%.0fs）' % timeout)
            if not self.update_pose():
                continue
            x, y, yaw = self.pose

            if self.goal_seq != my_seq:    # 手机又点了一个新点 → 放弃当前路径，改道去新目标
                self.get_logger().info('收到新目标 → 放弃当前路径，改道')
                return 'replan'

            # 最近航点索引（只看前若干个，避免跳点）
            dmin, idx = 1e9, 0
            for i, (wx, wy) in enumerate(wp):
                d = math.hypot(wx - x, wy - y)
                if d < dmin:
                    dmin, idx = d, i
            remain = sum(math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1])
                         for i in range(idx, len(wp) - 1))
            d_goal = math.hypot(wp[-1][0] - x, wp[-1][1] - y)
            self.last_d_goal = d_goal
            if d_goal < self.a.tol_xy:
                self.stop(0.3)
                self.get_logger().info('到达目标：距终点 %.3f m（离路径 %.3f m）'
                                       % (d_goal, dmin))
                return 'arrived'
            if dmin > 0.6:
                self.get_logger().warn('偏离路径 %.2f m → 重新规划' % dmin)
                return 'replan'

            # 前视点（沿路径往前走 lookahead 米）
            ld, i, acc = self.a.lookahead, idx, 0.0
            while i < len(wp) - 1 and acc < ld:
                acc += math.hypot(wp[i + 1][0] - wp[i][0], wp[i + 1][1] - wp[i][1])
                i += 1
            lx, ly = wp[i]
            alpha = math.atan2(ly - y, lx - x) - yaw
            while alpha > math.pi:
                alpha -= 2 * math.pi
            while alpha < -math.pi:
                alpha += 2 * math.pi

            # 目标方向偏差大 → 先原地转（更稳，也避免麦轮斜行）
            if abs(alpha) > math.radians(60):
                self.send(0.0, max(-self.wmax, min(self.wmax, 2.0 * alpha)))
                continue

            # 前向安全
            v_cap = self.vmax
            if self.scan_min_front is not None:
                if self.scan_min_front < self.a.stop_near:
                    self.stop(0.2)
                    if block_t0 is None:
                        block_t0 = time.time()
                        self.get_logger().warn('前方 %.2fm 有障碍 → 停车等待' % self.scan_min_front)
                    if time.time() - block_t0 > 3.0:
                        self.get_logger().warn('前方持续被堵 → 重新规划')
                        return 'replan'
                    continue
                elif self.scan_min_front < self.a.slow_near:
                    v_cap = min(v_cap, 0.06)
                    block_t0 = None
                else:
                    block_t0 = None

            kappa = 2.0 * math.sin(alpha) / max(0.05, ld)     # 纯跟踪曲率
            wz = max(-self.wmax, min(self.wmax, v_cap * kappa))
            v = v_cap * max(0.35, 1.0 - abs(wz) / self.wmax)  # 转弯减速
            # 终点附近减速，避免冲过
            if remain < 0.5:
                v = min(v, max(0.05, v_cap * remain / 0.5))
            self.send(v, wz)

    def turn_to(self, yaw_target):
        """原地转到目标朝向（用 map 位姿闭环）"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < 20.0:
            self.spin_once()
            self.check_alive()
            if not self.update_pose():
                continue
            err = yaw_target - self.pose[2]
            while err > math.pi:
                err -= 2 * math.pi
            while err < -math.pi:
                err += 2 * math.pi
            if abs(err) < math.radians(self.a.tol_yaw):
                break
            self.send(0.0, max(-self.wmax, min(self.wmax, 1.5 * err)))
        self.stop(0.2)


def main():
    ap = argparse.ArgumentParser(description='目标点自主导航（A* + 纯跟踪）')
    ap.add_argument('--goal', nargs='+', type=float, metavar='X Y [YAW_DEG]',
                    help='目标：x y（米，地图坐标系），可选朝向（度）')
    ap.add_argument('--start', nargs='+', type=float, metavar='X Y',
                    help='起点（--dry-run 时用；不给则用 TF 的当前位姿）')
    ap.add_argument('--listen', action='store_true', help='等 /goal_pose 话题')
    ap.add_argument('--map', default=MAP_DEFAULT, help='地图前缀（默认 %s）' % MAP_DEFAULT)
    ap.add_argument('--inflation', type=float, default=0.15,
                    help='障碍膨胀半径 m（默认 0.15，适配当前地图；车半宽 0.12，'
                         '地图补扫干净后可调大到 0.20~0.25）')
    ap.add_argument('--prefer-open', type=float, default=3.0,
                    help='"走宽处"偏好强度（默认 3；0=关 → 纯最短路径）。'
                         '实测 3 即到位：窄处占比 92%%→9%%；再往上收益递减、路径更长')
    ap.add_argument('--open-dist', type=float, default=0.8,
                    help='多宽算"宽敞" m（默认 0.8）：离障碍小于它就开始加价')
    ap.add_argument('--min-obstacle-cells', type=int, default=0,
                    help='把小于 N 格的障碍团簇当噪点抹掉（**默认 0 = 不抹**）。'
                         '⚠ 2026-09-21 实测更正：地图上的小点多是**椅腿/桌腿**等真实障碍，'
                         '调大等于把它们从图上删掉 → 路径会"变短变直"，但车可能直接撞上去；'
                         '除非能确认是残差，否则保持 0')
    ap.add_argument('--allow-unknown', action='store_true', help='允许穿过未知区（默认禁止）')
    ap.add_argument('--vmax', type=float, default=0.15, help='直行速度上限 m/s')
    ap.add_argument('--wmax', type=float, default=0.8, help='转向速度上限 rad/s')
    ap.add_argument('--lookahead', type=float, default=0.4, help='纯跟踪前视距离 m')
    ap.add_argument('--tol-xy', type=float, default=0.15, help='到点判定 m')
    ap.add_argument('--tol-yaw', type=float, default=10.0, help='到点朝向判定 度')
    ap.add_argument('--topic', default='/cmd_vel', help='速度话题（自测可换）')
    ap.add_argument('--dry-run', action='store_true', help='只规划 + 出路径图，不动车')
    ap.add_argument('--max-replan', type=int, default=5, help='最多重新规划次数')
    ap.add_argument('--wait-pose', type=float, default=30.0,
                    help='启动时等待 map→base_link 的秒数（默认 30；定位刚起/加载建图序列可能要十几秒）')
    ap.add_argument('--csv', metavar='OUT',
                    help='把轨迹写进 CSV（t,x,y,yaw,v,w,d_goal，≤5Hz）——N5 归档用，可直接叠到地图上画')
    ap.add_argument('--stop-near', type=float, default=STOP_NEAR,
                    help='前向扇区急停距离 m（默认 %.2f；做"人为挡路"安全测试可给大一点，如 0.30）'
                         % STOP_NEAR)
    ap.add_argument('--slow-near', type=float, default=SLOW_NEAR,
                    help='前向扇区减速距离 m（默认 %.2f）' % SLOW_NEAR)
    ap.add_argument('--sector-deg', type=float, default=SECTOR_DEG,
                    help='前向安全扇区半角 度（默认 %.0f）' % SECTOR_DEG)
    ap.add_argument('--scan-topic', default='/scan',
                    help='雷达话题（默认 /scan；自测可换测试话题，避免干扰真机 SLAM）')
    ap.add_argument('--goal-topic', default='/goal_pose',
                    help='目标点话题（默认 /goal_pose；**自测务必换测试话题**，'
                         '否则真机 nav 会收到你的测试目标并真的开过去）')
    ap.add_argument('--tf-parent', default='map',
                    help='位姿来源：父坐标系（默认 map；自测时可换成不冲突的坐标系）')
    ap.add_argument('--tf-child', default='base_link',
                    help='位姿来源：子坐标系（默认 base_link；自测时换成假底盘用的那个）')
    ap.add_argument('--smooth-iter', type=int, default=2,
                    help='拐角平滑迭代次数（Chaikin 切角；0=关闭。平滑后自动逐格碰撞复查，越界则回退原折线）')
    args = ap.parse_args()

    rclpy.init()
    node = GoalNav(args)
    m = node.m
    node.get_logger().info('地图：%s（%dx%d @%.3f m/px，膨胀 %.2fm%s）'
                           % (os.path.basename(args.map), m['w'], m['h'], m['res'],
                              args.inflation, '，允许未知区' if args.allow_unknown else ''))

    # ---------------- dry-run：只规划 ----------------
    if args.dry_run:
        if args.start:
            node.pose = (args.start[0], args.start[1], 0.0)
        else:
            t0 = time.time()
            while not node.update_pose() and time.time() - t0 < 5.0:
                node.spin_once()
            if node.pose is None:
                print('✗ dry-run 需要位姿：加 --start x y（或先起 SLAM 提供 TF）')
                node.destroy_node(); rclpy.shutdown(); return 1
        if not args.goal:
            print('✗ dry-run 需要 --goal x y')
            node.destroy_node(); rclpy.shutdown(); return 1
        path, why = node.plan(args.goal[:2])
        if path is None:
            print('✗ 规划失败：%s' % why)
            node.destroy_node(); rclpy.shutdown(); return 1
        sp = world_to_px(m, node.pose[0], node.pose[1])
        gp = world_to_px(m, args.goal[0], args.goal[1])
        out = os.path.join(os.path.dirname(m['prefix']),
                           os.path.basename(m['prefix']) + '_path.png')
        draw_overlay(m, node.trav, path, sp, gp, out)
        print('✓ 规划成功（%s）：%d 个点 → 抽稀后 %d 个航点，路径长 %.2f m'
              % (why, len(path), len(simplify_path(path, 2)),
                 path_length_m(m, path)))
        print('  起点 (%.2f, %.2f) → 终点 (%.2f, %.2f)｜路径图：%s'
              % (node.pose[0], node.pose[1], args.goal[0], args.goal[1], out))
        node.destroy_node(); rclpy.shutdown(); return 0

    # ---------------- 真导航 ----------------
    if args.goal:
        yaw = math.radians(args.goal[2]) if len(args.goal) > 2 else None
        node.goal = (args.goal[0], args.goal[1], yaw)
    # 等定位：位姿只在 update_pose() 里更新 → 必须每轮调用（只 spin_once 是永远等不到的）
    t0, nxt = time.time(), 5.0
    node.get_logger().info('等待定位：map→base_link（最多 %.0fs）...' % args.wait_pose)
    while rclpy.ok() and node.pose is None and time.time() - t0 < args.wait_pose:
        node.spin_once()
        node.update_pose()
        if node.pose is None and time.time() - t0 > nxt:
            print('  ... 仍在等 map→base_link（已 %.0fs / %.0fs）' % (time.time() - t0, args.wait_pose))
            nxt += 5.0
    if node.pose is None:
        print('✗ %.0fs 内拿不到 map→base_link。排查顺序：' % args.wait_pose)
        print('   ① 定位那套起了吗：robotnav（= localization 模式）；ps -ef | grep slam_toolbox')
        print('   ② TF 链完整吗：ros2 run tf2_tools view_frames.py（应有 map→odom→base_link）')
        print('   ③ 车摆在建图起点附近否（见 08 方案 N1「建图起点」）')
        node.destroy_node(); rclpy.shutdown(); return 1
    node.get_logger().info('当前位姿：x=%.2f y=%.2f yaw=%.0f°'
                           % (node.pose[0], node.pose[1], math.degrees(node.pose[2])))

    # ---------------- 主循环 ----------------
    # --listen：**连续运行**——到达一个点后回等待状态，手机再点一个就接着去下一个
    # 一次性 --goal 且没开 --listen：跑完这个点就退出（保持老行为）
    once = (node.goal is not None) and not args.listen
    if node.goal is None:
        node.get_logger().info('等待 %s ...（到达后继续等下一个；Ctrl-C 退出）' % args.goal_topic)

    cur_seq, tries = None, 0
    while rclpy.ok():
        # 1) 没目标就等（持续刷新位姿，否则 last_pose_t 过期会误报"位姿断流"）
        if node.goal is None:
            if once:
                break
            while node.goal is None and rclpy.ok():
                node.spin_once(0.2)
                node.update_pose()
            continue

        # 2) 换了目标（或第一次）→ 重规划计数清零
        if node.goal_seq != cur_seq:
            cur_seq = node.goal_seq
            tries = 0

        g = node.goal
        path, why = node.plan(g[:2])
        if path is None:
            node.get_logger().error('规划失败：%s' % why)
            if once:
                break
            node.goal = None          # listen：放弃这个点，继续等下一个（不退出）
            continue
        node.get_logger().info('规划成功（%s）：路径 %.2f m，%d 航点'
                               % (why, path_length_m(m, path), len(simplify_path(path, 2))))
        try:
            r = node.follow(path)
        except RuntimeError as e:
            node.get_logger().error('中止：%s' % e)
            if once:
                break
            node.goal = None
            continue
        except KeyboardInterrupt:
            node.get_logger().warn('用户中断')
            break

        if r == 'arrived':
            if g[2] is not None:
                node.get_logger().info('转到目标朝向 %.0f°' % math.degrees(g[2]))
                node.turn_to(g[2])
            node.get_logger().info('✓ 到达目标 (%.2f, %.2f)%s'
                                   % (g[0], g[1],
                                      '' if once else '，继续等下一个目标…'))
            if once:
                break
            node.goal = None          # listen：清掉当前目标，回到等待状态
            continue

        tries += 1
        if tries > args.max_replan:
            node.get_logger().error('重规划超过 %d 次 → 放弃目标 (%.2f, %.2f)，继续等下一个'
                                    % (args.max_replan, g[0], g[1]))
            if once:
                break
            node.goal = None
            continue
        node.get_logger().warn('重新规划（第 %d 次）' % tries)

    node.stop(0.5)
    if node.csv_f is not None:
        node.csv_f.close()
        print('轨迹已写入：%s（可用 plot_traj.py 叠到地图上看）' % args.csv)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
