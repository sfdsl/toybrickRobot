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
按键：q / Esc 退出
"""
import argparse
import math

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class MapViewer(Node):
    def __init__(self, view_range):
        super().__init__("map_viewer")
        self.view_range = view_range
        self.map_msg = None
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

    def on_map(self, msg):
        self.map_msg = msg

    def robot_pose(self):
        """返回 (x, y, yaw)；TF 不可用时返回 None"""
        try:
            t = self.buf.lookup_transform("map", "base_link", Time())
        except Exception:
            return None
        tr = t.transform.translation
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
        return tr.x, tr.y, yaw

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
    args = parser.parse_args()

    rclpy.init()
    node = MapViewer(args.range)
    win = "SLAM Map (q to quit)"
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            cv2.imshow(win, node.render())
            key = cv2.waitKey(50) & 0xFF
            if key in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
