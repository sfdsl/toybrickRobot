#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_editor —— 地图编辑核心：擦除假障碍 / 画虚拟墙，另存新版本（P2-A4）

⚠️ 先说清一个坑（决定本模块存在的意义）：
    nav2 的 `static_layer` 订阅的是 **/map**，而 /map 由 slam_toolbox **从
    posegraph 栅格化**发布 —— **不是 PGM**。所以把虚拟墙画进 `robot_map.pgm`
    对 nav2 **完全无效**（只影响 mapv / goal_nav / places）。

因此本模块的工作流是：

    ① 从 /map 抓一份 OccupancyGrid 副本（与定位坐标系**逐像素对齐**）
    ② 在副本上写笔刷：erase → 0（空闲）  wall → 100（障碍）
    ③ 导出 <prefix>.pgm + <prefix>.yaml（复用 save_map.save_pgm_yaml）
    ④ 由 nav2_map_server 加载该 yaml → /map_edited → static_layer 消费

**永不写 `robot_map.*`**（另存不覆盖原则，见 17 号文档 P2）。

用法：
    python3 map_editor.py --show          # 读 /map，打印尺寸/origin/resolution
    python3 map_editor.py --export /tmp/x # 读 /map 后原样导出（验证导出链路）

被 robot_ui 的 map_editor 服务复用：load_from_msg / apply_stroke / export。
"""
import argparse
import os
import sys
import time
import threading
from functools import wraps

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from save_map import save_pgm_yaml          # noqa: E402  复用同一套 PGM/YAML 落盘

try:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                           QoSReliabilityPolicy)
except ImportError as e:
    raise SystemExit(
        '导入 ROS 2 模块失败：%s\n提示：先 source 环境\n'
        '  source /opt/ros/humble/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

# 笔刷写入的栅格值（OccupancyGrid 约定：0=空闲 100=障碍 -1=未知）
FREE = 0
OCCUPIED = 100
MODES = ('erase', 'wall')
DEFAULT_EXPORT_PREFIX = os.path.expanduser('~/RobotCode/04_map/robot_map_clean')


def synchronized(method):
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self.lock:
            return method(self, *args, **kwargs)
    return locked


class MapEditor:
    """在 /map 的副本上做笔刷编辑，导出成 nav2 map_server 可直接加载的新版本。

    只碰内存里的栅格，**不发任何 ROS 话题**（不参与运动控制，纯编辑）。
    """

    def __init__(self):
        self.lock = threading.RLock()
        self._base_grid = None
        self._saved_grid = None
        self.msg = None          # 抓下来的 OccupancyGrid（用于带出 info：origin/resolution）
        self.grid = None         # np.int16 (h, w)，取值 -1 / 0..100
        self._strokes = []       # 撤销栈：每步 [(flat_index, old_value), ...]
        self.saved_as = None     # 最近一次导出的前缀
        self.edited = False      # 是否有未导出的改动

    # ---------------- 载入 ----------------
    @synchronized
    def load_from_msg(self, msg):
        """从一条 OccupancyGrid 载入（返回 (ok, 说明)）"""
        w, h = msg.info.width, msg.info.height
        if w <= 0 or h <= 0:
            return False, '地图尺寸非法：%dx%d' % (w, h)
        try:
            self.grid = np.array(msg.data, dtype=np.int16).reshape(h, w)
        except ValueError as e:
            return False, 'data 长度与尺寸不匹配：%s' % e
        self.msg = msg
        self._base_grid = self.grid.copy()
        self._saved_grid = self.grid.copy()
        self._strokes = []
        self.saved_as = None
        self.edited = False
        return True, '已载入 %dx%d（%.3f m/格）' % (w, h, msg.info.resolution)

    def load_from_topic(self, node, timeout=10.0, topic='/map'):
        """订阅 /map 拿一帧（transient_local，slam_toolbox 1 Hz 重发）"""
        box = {}
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        cached = getattr(node, 'raw_map_msg', None) or getattr(node, 'map_msg', None)
        if cached is not None:
            return self.load_from_msg(cached)
        subscription = node.create_subscription(OccupancyGrid, topic,
                                 lambda m: box.setdefault('m', m), qos)
        t0 = time.time()
        try:
            while time.time() - t0 < timeout:
                if hasattr(node, 'modes'):  # 网页节点已由后台 executor 处理回调。
                    time.sleep(0.05)
                else:
                    rclpy.spin_once(node, timeout_sec=0.1)
                if 'm' in box:
                    return self.load_from_msg(box['m'])
            return False, '超时 %.0fs 没收到 %s（slam_toolbox 没在跑？）' % (timeout, topic)
        finally:
            node.destroy_subscription(subscription)

    # ---------------- 坐标换算 ----------------
    def world_to_cell(self, x, y):
        """世界坐标 → (col, row)；越界返回 None"""
        if self.msg is None:
            return None
        info = self.msg.info
        col = int((x - info.origin.position.x) / info.resolution)
        row = int((y - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return None
        return col, row

    def cell_to_world(self, col, row):
        """(col, row) → 世界坐标（格子中心）"""
        info = self.msg.info
        return (info.origin.position.x + (col + 0.5) * info.resolution,
                info.origin.position.y + (row + 0.5) * info.resolution)

    # ---------------- 笔刷 ----------------
    @synchronized
    def apply_stroke(self, cells, mode, radius=1):
        """对 (col, row) 列表逐点半径 `radius` 的圆盘施加笔刷

        cells : [(col, row), ...]  —— 越界的点自动跳过
        mode  : 'erase' → 写 FREE(0)   'wall' → 写 OCCUPIED(100)
        radius: 半径（格），1 表示 3×3，2 表示 5×5

        返回 (ok, 说明)。整笔可 undo（一次 undo 撤销一次 apply_stroke）。
        """
        if self.grid is None:
            return False, '尚未载入地图'
        if mode not in MODES:
            return False, 'mode 必须是 %s 之一，收到 %r' % (MODES, mode)
        radius = max(1, int(radius))
        value = FREE if mode == 'erase' else OCCUPIED

        h, w = self.grid.shape
        undo_log = []
        r2 = radius * radius
        for col, row in cells:
            col, row = int(col), int(row)
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    if dr * dr + dc * dc > r2:
                        continue
                    c, r = col + dc, row + dr
                    if not (0 <= c < w and 0 <= r < h):
                        continue
                    flat = r * w + c
                    old = int(self.grid[r, c])
                    if old == value:
                        continue                     # 没变就不记，undo 栈不膨胀
                    undo_log.append((flat, old))
                    self.grid[r, c] = value

        if not undo_log:
            return False, '没有格子被改动（全在图外或已是目标值）'
        self._strokes.append(undo_log)
        self.edited = not np.array_equal(self.grid, self._saved_grid)
        return True, '已改 %d 格（%s, r=%d）' % (len(undo_log), mode, radius)

    @synchronized
    def undo(self):
        """撤销最近一次 apply_stroke"""
        if not self._strokes:
            return False, '没有可撤销的笔画'
        for flat, old in reversed(self._strokes.pop()):
            self.grid.flat[flat] = old
        self.edited = not np.array_equal(self.grid, self._saved_grid)
        return True, '已撤销（剩余 %d 笔）' % len(self._strokes)

    @synchronized
    def reset(self):
        """清空所有编辑，回到刚载入的状态"""
        if self._base_grid is None:
            return False, '尚未载入地图'
        self.grid = self._base_grid.copy()
        self._strokes = []
        self.edited = not np.array_equal(self.grid, self._saved_grid)
        return True, '已清空所有编辑'

    # ---------------- 导出 ----------------
    def _export_msg(self):
        """把编辑后的栅格装回一条 OccupancyGrid（info 原样带出）"""
        out = OccupancyGrid()
        out.header = self.msg.header
        out.info = self.msg.info
        out.data = self.grid.flatten().astype(np.int8).tolist()
        return out

    @synchronized
    def export(self, prefix=None):
        """导出 <prefix>.pgm + .yaml；**原子写**（tmp + fsync + replace）

        prefix 缺省用 DEFAULT_EXPORT_PREFIX。返回 (ok, 说明, [文件路径])。
        """
        if self.grid is None:
            return False, '尚未载入地图', []
        prefix = os.path.expanduser(prefix or DEFAULT_EXPORT_PREFIX)
        d = os.path.dirname(prefix)
        if d:
            os.makedirs(d, exist_ok=True)

        # 先写到临时前缀，再 rename 到最终名 —— 断电不会留下半个文件
        tmp_prefix = prefix + '.__tmp__'
        try:
            files = save_pgm_yaml(self._export_msg(), tmp_prefix)
        except Exception as e:
            return False, '导出失败：%s' % e, []

        final = []
        try:
            # save_pgm_yaml 写出的 yaml 里 image: 用的是 tmp 前缀的 basename，
            # rename 之后必须改回最终名，否则 map_server 找不到图。
            yml_tmp = tmp_prefix + '.yaml'
            with open(yml_tmp, encoding='utf-8') as f:
                body = f.read()
            body = body.replace(os.path.basename(tmp_prefix) + '.pgm',
                                os.path.basename(prefix) + '.pgm')
            with open(yml_tmp, 'w', encoding='utf-8') as f:
                f.write(body)
                f.flush()
                os.fsync(f.fileno())
            for ext in ('.pgm', '.yaml'):
                os.replace(tmp_prefix + ext, prefix + ext)
                final.append(prefix + ext)
        finally:
            for ext in ('.pgm', '.yaml'):
                if os.path.exists(tmp_prefix + ext):
                    os.unlink(tmp_prefix + ext)

        self.saved_as = prefix
        self._saved_grid = self.grid.copy()
        self.edited = False
        return True, '已导出（%d 笔编辑）' % len(self._strokes), final

    # ---------------- 给 SSE 用 ----------------
    @synchronized
    def status(self):
        if self.grid is None:
            return {'loaded': False, 'w': 0, 'h': 0, 'strokes': 0,
                    'saved_as': self.saved_as, 'map_id': None}
        info = self.msg.info
        return {
            'loaded': True,
            'w': int(info.width), 'h': int(info.height),
            'resolution': round(float(info.resolution), 4),
            'origin': [round(float(info.origin.position.x), 3),
                       round(float(info.origin.position.y), 3)],
            'strokes': len(self._strokes),
            'edited': self.edited,
            'saved_as': self.saved_as,
        }

    @staticmethod
    def active_prefix(base):
        """给 nav2_map_server 选地图：有 clean 版就用 clean，否则用原始

        base 例：~/RobotCode/04_map/robot_map
        """
        base = os.path.expanduser(base)
        clean = base + '_clean'
        return clean if os.path.isfile(clean + '.yaml') else base


def main():
    ap = argparse.ArgumentParser(description='地图编辑核心（擦除/画墙，另存新版本）')
    ap.add_argument('--show', action='store_true', help='读 /map 打印尺寸与 origin')
    ap.add_argument('--export', metavar='PREFIX',
                    help='原样导出一份（验证导出链路）')
    ap.add_argument('--active', metavar='BASE',
                    help='打印 nav2_map_server 该加载哪个前缀（clean 优先）')
    args = ap.parse_args()

    if args.active:
        print('应加载：%s' % MapEditor.active_prefix(args.active))
        return

    rclpy.init()
    node = rclpy.create_node('map_editor_cli')
    ed = MapEditor()
    try:
        ok, msg = ed.load_from_topic(node, timeout=10.0)
        print(('✓ ' if ok else '✗ ') + msg)
        if not ok:
            raise SystemExit(1)
        if args.show:
            print('  尺寸   : %dx%d' % (ed.status()['w'], ed.status()['h']))
            print('  分辨率 : %.3f m/格' % ed.status()['resolution'])
            print('  origin : %s' % ed.status()['origin'])
            occ = int((ed.grid >= 65).sum())
            free = int((ed.grid == 0).sum())
            unk = int((ed.grid == -1).sum())
            print('  栅格   : 障碍 %d / 空闲 %d / 未知 %d' % (occ, free, unk))
        if args.export:
            ok, msg, files = ed.export(args.export)
            print(('✓ ' if ok else '✗ ') + msg)
            for f in files:
                print('    %s' % f)
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
