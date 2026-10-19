#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时 SLAM 地图查看器（OpenCV）—— rviz2 的轻量替代

背景：本机 Mali GPU 只有 Wayland/EGL 驱动，rviz2 的渲染引擎 OGRE 仅支持
GLX（X11），故 rviz2 无法在本机运行；此脚本用 OpenCV 渲染（零 GL 依赖）。

订阅：
    /map     （OccupancyGrid，transient_local）
    /tf      （用于画车辆位置与朝向：map → base_link）

画面：
    白色 = 空闲区域、黑色 = 障碍、灰色 = 未知
    红色三角 = 车辆当前位置与朝向
    以车辆为中心的视窗（默认 8 米，-r 可调）

用法：
    python3 map_viewer.py           # 以车为中心 8 米视窗
    python3 map_viewer.py -r 12     # 视野 12 米
    python3 map_viewer.py --costmap # 叠加 nav2 膨胀层（看"这条缝过不过得去"）
按键：q / Esc 退出

--costmap 的三档颜色（nav2 代价 0~100）：
    黄 1~49  轻膨胀（车中心能过，但离障碍近）
    橙 50~99 重膨胀（代价高，规划器倾向绕开）
    红 100   致命（车中心根本到不了 —— 红区连成一片就说明这条缝过不去）
"""
import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


# /global_costmap/costmap 发布的是 **0~100** 的 OccupancyGrid（nav2 把内部 0~254 折算过）：
#   100 = 障碍本体(LETHAL) 或 inscribed 区（机器人中心进不去）；其余按 ratio 折算到 1~99。
# 所以画红的阈值只能是 100。注意 nav2_params 里的 lethal_cost_threshold=130 是**内部量纲**，
# 换算到发布值约等于 52 —— 也就是说**橙色区(50~99)规划器大多已判不可通行**。
LETHAL = 100


class MapViewer(Node):
    def __init__(self, view_range, show_costmap=False):
        super().__init__("map_viewer")
        self.view_range = view_range
        self.map_msg = None
        self.costmap_msg = None      # --costmap：/global_costmap/costmap（含膨胀层）
        self.show_costmap = show_costmap
        # 最近一次 render() 的视窗参数 (x0, y1, 像素/米, size)，供"画面像素→世界坐标"反算
        # （手机网页点选目标用；详见 phone_teleop.py 的 /goal 接口）
        self.last_view = None
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qos)
        self.get_logger().info("等待 /map ...（确认 slam_toolbox 正在运行）")
        if show_costmap:
            # 用 VOLATILE 订阅：nav2 的发布者若为 TRANSIENT_LOCAL 也能兼容；
            # 反过来（本端请求 TRANSIENT_LOCAL、对端是 VOLATILE）会一条都收不到。
            cqos = QoSProfile(depth=1,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE)
            self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                     self.on_costmap, cqos)
            self.get_logger().info(
                "膨胀层已开：黄=轻膨胀 橙=重膨胀 红=致命（红区连成片 = 过不去）")

    def on_map(self, msg):
        self.map_msg = msg

    def on_costmap(self, msg):
        self.costmap_msg = msg

    def robot_pose(self, max_age=None):
        """返回 (x, y, yaw)；可选拒绝过期的定位 TF。"""
        try:
            if max_age is not None:
                localization = self.buf.lookup_transform("map", "odom", Time())
                age = (self.get_clock().now() - Time.from_msg(
                    localization.header.stamp)).nanoseconds / 1e9
                if age > max_age or age < -1.0:
                    return None
            t = self.buf.lookup_transform("map", "base_link", Time())
        except Exception:
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        return tr.x, tr.y, yaw

    def _overlay_costmap(self, canvas, x0, y1, s, size):
        """把 /global_costmap/costmap（含膨胀）半透明叠到画面上。

        判读方法：**红色（代价 100）连成一片的地方，车的中心到不了**。
        两箱之间那条缝如果被红色完全填满，就说明物理上过不去 ——
        对应"净缝宽 < 0.56 m"那个结论，比量尺子直观。
        """
        cm = self.costmap_msg
        res = cm.info.resolution
        ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
        w, h = cm.info.width, cm.info.height
        grid = np.array(cm.data, dtype=np.int16).reshape(h, w)

        span = size / s
        gx0 = max(0, int((x0 - ox) / res) - 1)
        gx1 = min(w, int((x0 + span - ox) / res) + 2)
        gy0 = max(0, int((y1 - span - oy) / res) - 1)
        gy1 = min(h, int((y1 - oy) / res) + 2)
        if gx1 <= gx0 or gy1 <= gy0:
            return

        sub = grid[gy0:gy1, gx0:gx1]
        layer = np.zeros(sub.shape + (3,), np.uint8)
        alpha = np.zeros(sub.shape, np.float32)
        band = (sub > 0) & (sub < 50)                    # 轻膨胀 黄
        layer[band] = (0, 220, 255)
        alpha[band] = 0.22
        band = (sub >= 50) & (sub < LETHAL)              # 重膨胀 橙
        layer[band] = (0, 140, 255)
        alpha[band] = 0.45
        band = sub >= LETHAL                             # 致命 红 = 规划器真正判不可通行的线
        layer[band] = (0, 0, 255)
        alpha[band] = 0.72
        if not alpha.any():
            return

        sx0w = ox + gx0 * res
        sy1w = oy + gy1 * res
        px0 = int(round((sx0w - x0) * s))
        py0 = int(round((y1 - sy1w) * s))
        W = int(round((gx1 - gx0) * res * s))
        H = int(round((gy1 - gy0) * res * s))
        if W <= 0 or H <= 0:
            return
        layer = cv2.resize(np.flipud(layer), (W, H), interpolation=cv2.INTER_NEAREST)
        alpha = cv2.resize(np.flipud(alpha), (W, H), interpolation=cv2.INTER_NEAREST)

        dx0, dy0 = max(px0, 0), max(py0, 0)
        dx1, dy1 = min(px0 + W, size), min(py0 + H, size)
        if dx1 <= dx0 or dy1 <= dy0:
            return
        a = alpha[dy0 - py0:dy1 - py0, dx0 - px0:dx1 - px0][..., None]
        lay = layer[dy0 - py0:dy1 - py0, dx0 - px0:dx1 - px0].astype(np.float32)
        roi = canvas[dy0:dy1, dx0:dx1].astype(np.float32)
        canvas[dy0:dy1, dx0:dx1] = (roi * (1.0 - a) + lay * a).astype(np.uint8)

    def render(self, size=720, view_range=None, center=None, offset=None):
        """渲染一帧

        view_range : 视野米数（默认 self.view_range）——网页缩放就是改它
        center     : 视窗中心的世界坐标 (x, y)；不给则跟随车辆（无 TF 时用地图中心）
        offset     : 在基准中心上再叠加 (dx, dy)（手机平移用；车动时视野仍跟着车）
        """
        view_range = self.view_range if view_range is None else float(view_range)
        msg = self.map_msg
        if msg is None:
            self.last_view = None
            img = np.full((size, size, 3), 30, np.uint8)
            cv2.putText(img, "waiting /map ...", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return img

        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        w, h = msg.info.width, msg.info.height
        grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

        # 视窗（世界坐标，米）
        pose = self.robot_pose()
        half = view_range / 2.0
        if center is not None:
            cx_w, cy_w = center
        elif pose is not None:
            cx_w, cy_w = pose[0], pose[1]
        else:
            cx_w = ox + w * res / 2.0
            cy_w = oy + h * res / 2.0
        if offset is not None:              # 手机平移：在基准中心上叠加（车动时视野仍跟着车）
            cx_w += float(offset[0])
            cy_w += float(offset[1])
        x0, y0 = cx_w - half, cy_w - half
        x1, y1 = cx_w + half, cy_w + half
        s = size / view_range               # 像素 / 米（必须用本次的视野，缩放才对）
        self.last_view = (x0, y1, s, size)  # 供画面像素 → 世界坐标反算（手机点选目标用）

        canvas = np.full((size, size, 3), 30, np.uint8)

        # 视窗覆盖的地图格子范围（含 1 格余量）
        gx0 = max(0, int((x0 - ox) / res) - 1)
        gx1 = min(w, int((x1 - ox) / res) + 2)
        gy0 = max(0, int((y0 - oy) / res) - 1)
        gy1 = min(h, int((y1 - oy) / res) + 2)
        if gx1 > gx0 and gy1 > gy0:
            sub = grid[gy0:gy1, gx0:gx1]
            subimg = np.full(sub.shape + (3,), 128, np.uint8)   # 未知 灰
            subimg[sub == 0] = (255, 255, 255)                   # 空闲 白
            subimg[sub >= 65] = (0, 0, 0)                        # 障碍 黑
            subimg = np.flipud(subimg)                           # 图像行从上方开始

            sx0w = ox + gx0 * res            # 子图左上角对应世界坐标
            sy1w = oy + gy1 * res            # （顶部 = y 大侧）
            px0 = int(round((sx0w - x0) * s))
            py0 = int(round((y1 - sy1w) * s))
            W = int(round((gx1 - gx0) * res * s))
            H = int(round((gy1 - gy0) * res * s))
            if W > 0 and H > 0:
                patch = cv2.resize(subimg, (W, H),
                                   interpolation=cv2.INTER_NEAREST)
                dx0, dy0 = max(px0, 0), max(py0, 0)
                dx1, dy1 = min(px0 + W, size), min(py0 + H, size)
                if dx1 > dx0 and dy1 > dy0:
                    canvas[dy0:dy1, dx0:dx1] = \
                        patch[dy0 - py0:dy1 - py0, dx0 - px0:dx1 - px0]

        # 膨胀层（可选，叠在静态地图之上、车辆之下）
        if self.show_costmap and self.costmap_msg is not None:
            self._overlay_costmap(canvas, x0, y1, s, size)

        # 车辆位置（红色三角）
        if pose is not None:
            x, y, yaw = pose
            px = int((x - x0) * s)
            py = int((y1 - y) * s)
            L = 16
            pts = []
            for a in (0.0, 2.5, -2.5):
                ang = yaw + a
                pts.append((int(px + L * math.cos(ang)),
                            int(py - L * math.sin(ang))))
            cv2.polylines(canvas, [np.array(pts)], True, (0, 0, 255), 3)

        # HUD
        cv2.putText(canvas, "range %.1fm | res %.3f | %dx%d"
                    % (view_range, res, w, h),
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (200, 200, 200), 1)
        if pose is None:
            cv2.putText(canvas, "no TF map->base_link", (10, 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 1)
        return canvas


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--range", type=float, default=8.0,
                        help="以车为中心的视野范围（米）")
    parser.add_argument("--costmap", action="store_true",
                        help="叠加 nav2 全局代价地图的膨胀层（黄/橙/红）")
    parser.add_argument("--save", metavar="PNG",
                        help="不开窗口：等 --wait 秒后渲染一帧写成 PNG 并退出"
                             "（SSH / 无显示环境用）")
    parser.add_argument("--wait", type=float, default=6.0,
                        help="--save 模式下等多少秒收数据（默认 6）")
    parser.add_argument("--stream", metavar="PNG",
                        help="持续渲染到同一个 PNG（配合 http.server 供浏览器刷新看）")
    parser.add_argument("--period", type=float, default=1.5,
                        help="--stream 模式的刷新周期秒（默认 1.5）")
    args = parser.parse_args()

    rclpy.init()
    node = MapViewer(args.range, args.costmap)

    if args.save:
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < args.wait:
            rclpy.spin_once(node, timeout_sec=0.1)
        img = node.render(size=1024)
        cv2.imwrite(args.save, img)
        print("已写出 %s ｜ /map=%s ｜ /global_costmap/costmap=%s"
              % (args.save,
                 "有" if node.map_msg else "无",
                 "有" if node.costmap_msg else ("无" if args.costmap else "未开")))
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return

    if args.stream:
        print("流式渲染 → %s（每 %.1fs 一帧，Ctrl-C 退出）" % (args.stream, args.period))
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                t0 = time.time()
                cv2.imwrite(args.stream, node.render(size=1024))
                while time.time() - t0 < args.period:      # 周期内继续 spin，数据不积压
                    rclpy.spin_once(node, timeout_sec=0.05)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass
        return

    win = "SLAM Map (q to quit)"
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            try:
                cv2.imshow(win, node.render())
            except cv2.error as e:
                # SSH / 无图形会话 → GTK 后端初始化失败。别甩一屏 traceback，直接指路。
                print("\n✗ 打不开窗口：当前环境没有图形显示服务（多为 SSH 未开 X11）")
                print("  %s" % str(e).strip().splitlines()[0])
                print("  → 改用无头模式存图，再拉到电脑上看：")
                print("     python3 map_viewer.py --costmap --save /tmp/map.png -r 8")
                raise SystemExit(1)
            key = cv2.waitKey(50) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
