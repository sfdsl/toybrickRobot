#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map_overlay —— 给 nav2 提供 /map_edited：实时地图 + 已保存的编辑覆盖层（P2-A4）

为什么不用 nav2_map_server（原计划）：
    nav2_map_server 发的是**静态快照**，而探索模式下 slam_toolbox 的 /map 是
    **持续增长**的。用快照会让全局规划器看不到新探索的区域，而 SmacPlanner2D
    配的是 `allow_unknown: false`（不许穿过未知），探索会直接卡死 —— 见 17 号
    文档 P2「探索 = 导航 + 建图闭环」。

本节点做法：
    订阅 /map（实时，transient_local）
        ↓ 叠加已保存的编辑层（robot_map_clean.pgm，若有）
    发布 /map_edited
        ↓
    nav2 global_costmap.static_layer 消费（map_topic: /map_edited）

- 编辑层按**世界坐标**对齐到实时地图（两者 origin/resolution 可不同）
- 没有编辑层时就是纯透传 → 与原行为完全一致（一键回退：把 map_topic 改回 /map）
- 编辑层只改**已知**格子（PGM 里 205=未知 不动），所以不会把新探索区封死

用法：
    python3 map_overlay.py                     # 自动找 robot_map_clean.pgm
    python3 map_overlay.py --overlay /path/x   # 指定编辑层前缀（.pgm/.yaml）
    python3 map_overlay.py --none              # 纯透传（等价于回退到 /map）
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                           QoSReliabilityPolicy)
except ImportError as e:
    raise SystemExit(
        '导入 ROS 2 模块失败：%s\n提示：先 source 环境\n'
        '  source /opt/ros/humble/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

DEFAULT_BASE = os.path.expanduser('~/RobotCode/04_map/robot_map')
OUT_TOPIC = '/map_edited'


def load_overlay(prefix):
    """读 <prefix>.pgm + .yaml → (grid int16 (h,w), res, ox, oy)；失败返回 None

    PGM 约定（同 save_map.save_pgm_yaml）：<100 障碍、>200 空闲、中间=未知。
    """
    prefix = os.path.expanduser(prefix)
    pgm, yml = prefix + '.pgm', prefix + '.yaml'
    if not (os.path.isfile(pgm) and os.path.isfile(yml)):
        return None
    with open(pgm, 'rb') as f:
        if f.readline().strip() != b'P5':
            return None
        line = f.readline().strip()
        while line.startswith(b'#'):
            line = f.readline().strip()
        w, h = map(int, line.split())
        f.readline()                                  # 255
        raw = np.frombuffer(f.read(w * h), dtype=np.uint8)
    if raw.size != w * h:
        return None
    img = raw.reshape(h, w)
    grid = np.full((h, w), -1, np.int16)               # 未知
    # slam_toolbox 保存图使用 0=障碍、205=未知、254=空闲。
    # 205 不能误判为空闲，否则编辑层会抹掉未知区的规划约束。
    grid[img > 210] = 0                                # 空闲
    grid[img < 100] = 100                              # 障碍
    grid = np.flipud(grid)                             # 图片行从上到下 → 栅格行从下到上

    res = 0.05
    ox = oy = 0.0
    try:
        with open(yml, encoding='utf-8') as f:
            for line in f:
                s = line.strip()
                if s.startswith('resolution:'):
                    res = float(s.split(':', 1)[1])
                elif s.startswith('origin:'):
                    nums = s.split('[', 1)[1].split(']')[0].split(',')
                    ox, oy = float(nums[0]), float(nums[1])
    except Exception:
        return None
    return grid, res, ox, oy


class MapOverlay:
    def __init__(self, overlay=None, period=2.0):
        self.pub = None
        self.overlay = overlay            # (grid, res, ox, oy) 或 None
        self.n = 0
        self._last_raw = None             # 缓存最近一帧 /map，供定时重发
        self._last_out = None             # 叠加后的结果（避免每拍重算）
        self.period = period
        self.live_received = False

    def set_overlay(self, overlay):
        self.overlay = overlay
        self._last_out = None             # 编辑层变了，重算

    def on_map(self, msg):
        self.live_received = True
        self._last_raw = msg
        self._last_out = None
        self._emit()

    def tick(self):
        """定时重发（默认 2 s 一次）。

        ⚠️ 为什么必须定时重发，而不能靠 TRANSIENT_LOCAL latch：
           2026-10-20 实测——/map 的发布者是 TRANSIENT_LOCAL + RELIABLE，
           但**晚加入的订阅者 60 s 一帧都收不到**（写端历史里那条 sample
           已经丢了，多半是被一堆死订阅者的 ACK 拖住/挤掉，与之前
           "/cmd_vel 有 7 个僵尸发布者"是同一类问题）。
           于是 mapv / mapweb / 探索屏 / 编辑屏 只要晚于 slam 启动就永远黑屏。
           定时重发让"晚到"和"早到"没有区别，彻底绕开这个坑。
        """
        if self._last_raw is not None:
            self._emit()

    def _emit(self):
        out = self._last_out
        if out is None:
            out = self._last_raw
            if self.overlay is not None:
                out = self._blend(self._last_raw, self.overlay)
            self._last_out = out
        if self.pub is not None and out is not None:
            self.pub.publish(out)
            self.n += 1

    @staticmethod
    def _blend(msg, ov):
        """把编辑层按世界坐标叠到实时地图上；只改实时地图里**已知**的格子"""
        g_ov, r_ov, ox_ov, oy_ov = ov
        info = msg.info
        w, h = info.width, info.height
        live = np.array(msg.data, dtype=np.int16).reshape(h, w)
        out = live.copy()

        rows, cols = g_ov.shape
        # 编辑层每个格子的中心世界坐标 → 实时地图格子
        cs = np.arange(cols)
        rs = np.arange(rows)
        wx = ox_ov + (cs + 0.5) * r_ov
        wy = oy_ov + (rs + 0.5) * r_ov
        li = np.floor((wx - info.origin.position.x) / info.resolution).astype(int)
        lj = np.floor((wy - info.origin.position.y) / info.resolution).astype(int)
        ok_i = (li >= 0) & (li < w)
        ok_j = (lj >= 0) & (lj < h)

        src = g_ov[:, :][:, ok_i][ok_j, :]             # 只取落在图内的
        if src.size == 0:
            return msg
        ti = li[ok_i]
        tj = lj[ok_j]
        # 只覆盖"编辑层明确写了值"的格子（-1 = 未知，保持实时地图原值）
        mask = src >= 0
        if not mask.any():
            return msg
        TI, TJ = np.meshgrid(ti, tj, indexing='ij')
        # meshgrid 形状是 (len(ti), len(tj))，要转置对齐 src 的 (row=j, col=i)
        TI = TI.T
        TJ = TJ.T
        out[TJ[mask], TI[mask]] = src[mask]

        out_msg = OccupancyGrid()
        out_msg.header = msg.header
        out_msg.info = msg.info
        out_msg.data = out.flatten().astype(np.int8).tolist()
        return out_msg


def base_map_message(prefix):
    """定位模式兜底：从已保存的地图生成与 /map 相同坐标系的栅格。"""
    loaded = load_overlay(prefix)
    if loaded is None:
        return None
    grid, resolution, ox, oy = loaded
    out = OccupancyGrid()
    out.header.frame_id = 'map'
    out.info.resolution = resolution
    out.info.width = grid.shape[1]
    out.info.height = grid.shape[0]
    out.info.origin.position.x = ox
    out.info.origin.position.y = oy
    out.info.origin.orientation.w = 1.0
    out.data = grid.flatten().astype(np.int8).tolist()
    return out


def main():
    ap = argparse.ArgumentParser(description='实时地图 + 编辑覆盖层 → /map_edited')
    ap.add_argument('--overlay', metavar='PREFIX',
                    help='编辑层前缀（.pgm/.yaml）；默认自动找 <robot_map>_clean')
    ap.add_argument('--none', action='store_true', help='纯透传（等价于不启用编辑）')
    ap.add_argument('--out', default=OUT_TOPIC, help='输出话题（默认 %s）' % OUT_TOPIC)
    ap.add_argument('--period', type=float, default=2.0,
                    help='定时重发间隔秒（默认 2.0）；晚加入的订阅者靠它拿数据，'
                         '不依赖 TRANSIENT_LOCAL latch（实测 latch 在本机不可靠）')
    ap.add_argument('--mode', choices=('mapping', 'localization'), default='localization',
                    help='定位模式可从已保存的地图初始化；建图模式只用实时地图')
    ap.add_argument('--base-map', default=DEFAULT_BASE,
                    help='定位模式的地图兜底前缀')
    args = ap.parse_args()

    overlay = None
    if not args.none:
        prefix = args.overlay or (DEFAULT_BASE + '_clean')
        overlay = load_overlay(prefix)
        if overlay is None:
            print('提示：没有可用的编辑层 %s → 纯透传 /map' % os.path.basename(prefix))
        else:
            print('✓ 已载入编辑层 %s（%dx%d）' %
                  (os.path.basename(prefix), overlay[0].shape[1], overlay[0].shape[0]))

    rclpy.init()
    node = rclpy.create_node('map_overlay')
    qos_in = QoSProfile(depth=1,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    qos_out = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    mo = MapOverlay(overlay, period=args.period)
    mo.pub = node.create_publisher(OccupancyGrid, args.out, qos_out)

    def on_live_map(msg):
        if not mo.live_received:
            node.get_logger().info('已接收 /map：%dx%d，实时地图接管' %
                                   (msg.info.width, msg.info.height))
        mo.on_map(msg)

    node.create_subscription(OccupancyGrid, '/map', on_live_map, qos_in)
    if args.mode == 'localization':
        fallback = base_map_message(args.base_map)
        if fallback is None:
            node.get_logger().warn('已保存地图不可用：等待 slam_toolbox /map 或 dynamic_map 服务')
        else:
            mo._last_raw = fallback
            mo._emit()
            node.get_logger().info('已用 %s 初始化 %s，等待实时地图接管' %
                                   (args.base_map, args.out))
    # late /map 的瞬态历史若未交付，可从 slam_toolbox 的 GetMap 服务取当前图。
    # 异步调用保证服务尚未启动时不会堵住 /map 订阅回调。
    from nav_msgs.srv import GetMap
    client = node.create_client(GetMap, '/slam_toolbox/dynamic_map')
    pending = [None]

    def poll_map_service():
        if mo.live_received or pending[0] is not None or not client.service_is_ready():
            return
        future = client.call_async(GetMap.Request())
        pending[0] = future

        def on_result(done):
            pending[0] = None
            try:
                result = done.result()
                if result and result.map.info.width and result.map.info.height:
                    mo.on_map(result.map)
                    node.get_logger().info('已从 /slam_toolbox/dynamic_map 获取实时地图')
            except Exception as exc:
                node.get_logger().warn('获取实时地图失败：%s' % exc)

        future.add_done_callback(on_result)

    node.create_timer(args.period, poll_map_service)
    node.create_timer(args.period, mo.tick)          # 定时重发，见 MapOverlay.tick

    def warn_if_empty():
        if mo._last_raw is None:
            node.get_logger().error('/map_edited 尚无地图：/map 与 dynamic_map 均未提供有效数据',
                                    throttle_duration_sec=10.0)

    node.create_timer(args.period, warn_if_empty)
    node.get_logger().info('map_overlay: /map → %s（编辑层 %s，每 %.1fs 重发）'
                           % (args.out, '有' if overlay else '无（透传）', args.period))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
