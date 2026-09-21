#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时扫描点云查看器（ROS 版，OpenCV）—— rviz2 的轻量替代

背景：本机 Mali GPU 只有 Wayland/EGL 驱动，rviz2 无法运行；
此脚本用 OpenCV 渲染（零 GL 依赖），订阅 ROS 话题 /scan 显示点云。

与 lidar_viewer.py 的区别（重要，避免串口打架）：
  - lidar_viewer.py：直连雷达串口（无 ROS）——**雷达节点未运行时**才能用
  - 本脚本：订阅 /scan 话题——与 `ros2 launch ldlidar ...` 同时运行，
    不碰串口（串口由 ldlidar 节点独占）

画面（右手系，正前方朝上）：
    同心圆为距离环（默认每 1 m 一圈），顶部箭头为 0°（正前方）
    点颜色按距离：近=红，远=绿
    HUD：扫描频率 / 一圈点数 / 最近障碍（距离+方位）

用法：
    python3 scan_viewer.py                 # 订阅 /scan，量程 12 m
    python3 scan_viewer.py -t /scan -r 6   # 指定话题 / 量程
    python3 scan_viewer.py --headless      # 无显示器：终端打印统计
按键：q / Esc 退出；+ / - 缩放量程。
"""
import argparse
import math
import os
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

WIN = "ROS scan (q to quit)"


def setup_display():
    """SSH / IDE 终端里没有 DISPLAY 时，尝试自动对接本机 Wayland 桌面的 Xwayland。

    GNOME(mutter) 下 Xwayland 的授权文件每次登录都随机命名，
    旧的 ~/.Xauthority 会失效报 "No protocol specified"，所以这里动态取。
    返回 True 表示 X 环境可用，可以正常开窗。
    """
    if os.environ.get("DISPLAY"):
        return True
    xauth_dir = "/run/user/%d" % os.getuid()
    try:
        cands = sorted(os.path.join(xauth_dir, n) for n in os.listdir(xauth_dir)
                       if n.startswith(".mutter-Xwaylandauth."))
    except OSError:
        cands = []
    if not cands or not os.path.exists("/tmp/.X11-unix/X0"):
        return False
    os.environ["DISPLAY"] = ":0"
    os.environ["XAUTHORITY"] = cands[0]
    print("自动对接桌面显示环境 DISPLAY=:0 (%s)" % os.path.basename(cands[0]))
    return True


class ScanViewer(Node):
    def __init__(self, topic):
        super().__init__("scan_viewer")
        self.scan = None                      # (angles_deg, ranges) 已滤除无效点
        self.last_rx = 0.0                    # 最近一帧到达时刻
        self.stamps = deque(maxlen=8)         # 用于测实际到达频率
        self.create_subscription(LaserScan, topic, self.on_scan,
                                 qos_profile_sensor_data)
        self.get_logger().info("订阅 %s（需雷达节点已运行）…" % topic)

    def on_scan(self, msg):
        ang = np.degrees(msg.angle_min +
                         np.arange(len(msg.ranges)) * msg.angle_increment)
        rng = np.asarray(msg.ranges, dtype=np.float32)
        valid = (np.isfinite(rng) & (rng > max(msg.range_min, 0.01))
                 & (rng <= msg.range_max))
        self.scan = (ang[valid], rng[valid])
        now = time.time()
        self.last_rx = now
        self.stamps.append(now)

    def scan_hz(self):
        if len(self.stamps) < 2:
            return 0.0
        dt = self.stamps[-1] - self.stamps[0]
        return (len(self.stamps) - 1) / dt if dt > 1e-6 else 0.0

    def fresh_scan(self, timeout=1.5):
        """太久没新帧（雷达未启动/已停）返回 None，避免显示陈旧画面"""
        if self.scan is None or time.time() - self.last_rx > timeout:
            return None
        return self.scan


def polar_to_px(angle_deg, dist_m, cx, cy, scale):
    """右手系角度(正前方0, 逆时针+) -> 屏幕像素(前=上, 左=左)"""
    a = math.radians(angle_deg)
    return int(cx - dist_m * scale * math.sin(a)), \
           int(cy - dist_m * scale * math.cos(a))


def render(scan, max_range, hz, size=640):
    """把一圈点云画成俯视图，返回 BGR 图像"""
    img = np.zeros((size, size, 3), np.uint8)
    cx = cy = size // 2
    scale = (size / 2 - 30) / max_range

    # 距离环 + 刻度
    for r in range(1, int(max_range) + 1):
        cv2.circle(img, (cx, cy), int(r * scale), (60, 60, 60), 1)
        cv2.putText(img, "%dm" % r, (cx + 4, cy - int(r * scale) + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 90, 90), 1)
    # 十字基准线
    cv2.line(img, (cx, 0), (cx, size), (40, 40, 40), 1)
    cv2.line(img, (0, cy), (size, cy), (40, 40, 40), 1)
    # 正前方箭头
    cv2.arrowedLine(img, (cx, cy), (cx, 25), (0, 200, 200), 2, tipLength=0.15)
    cv2.putText(img, "FRONT", (cx + 6, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 200, 200), 1)
    cv2.circle(img, (cx, cy), 4, (0, 200, 200), -1)

    if scan is None:
        cv2.putText(img, "waiting /scan ...", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return img

    ang, dis = scan
    # 画点：近红远绿
    for a, d in zip(ang, dis):
        if d > max_range:
            continue
        px, py = polar_to_px(a, d, cx, cy, scale)
        ratio = min(d / max_range, 1.0)
        color = (0, int(255 * ratio), int(255 * (1 - ratio)))
        cv2.circle(img, (px, py), 2, color, -1)

    # 最近障碍
    hud = ["scan %.1f Hz | points %d" % (hz, len(ang))]
    if len(dis):
        i = int(np.argmin(dis))
        a = ang[i] - 360.0 if ang[i] > 180.0 else ang[i]   # 显示为 -180~180
        hud.append("nearest %.2f m @ %.1f deg" % (dis[i], a))
    for k, line in enumerate(hud):
        cv2.putText(img, line, (10, 25 + 22 * k),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


def main():
    ap = argparse.ArgumentParser(description="ROS 扫描点云查看器（订阅 /scan）")
    ap.add_argument("-t", "--topic", default="/scan", help="扫描话题")
    ap.add_argument("-r", "--max-range", type=float, default=12.0,
                    help="量程刻度 m")
    ap.add_argument("--headless", action="store_true",
                    help="不开窗，终端打印统计")
    args = ap.parse_args()

    setup_display()
    headless = args.headless or not os.environ.get("DISPLAY")

    rclpy.init()
    node = ScanViewer(args.topic)
    max_range = args.max_range
    last_stat = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if headless:
                if time.time() - last_stat >= 1.0:
                    last_stat = time.time()
                    scan = node.fresh_scan()
                    if scan is None:
                        print("等待 /scan ...（雷达节点是否在运行？）")
                    else:
                        ang, dis = scan
                        if len(dis):
                            i = int(np.argmin(dis))
                            a = ang[i] - 360.0 if ang[i] > 180.0 else ang[i]
                            print("scan %.1f Hz | %d 点 | 最近 %.2f m @ %.1f°"
                                  % (node.scan_hz(), len(ang), dis[i], a))
                        else:
                            print("scan %.1f Hz | 无有效回波"
                                  % node.scan_hz())
                time.sleep(0.05)
                continue

            cv2.imshow(WIN, render(node.fresh_scan(), max_range,
                                   node.scan_hz()))
            key = cv2.waitKey(30) & 0xFF
            if key in (27, ord("q")):
                break
            if key in (ord("+"), ord("=")):
                max_range = max(1.0, max_range - 1.0)
            if key == ord("-"):
                max_range = min(12.0, max_range + 1.0)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    print("已退出。")


if __name__ == "__main__":
    main()
