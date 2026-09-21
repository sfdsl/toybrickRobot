#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""订阅 /map 一次，保存为 PGM + YAML（与 nav2 map_server 格式兼容）

用法：
    python3 map_saver.py [输出前缀]     # 默认 map
    # 生成 map.pgm + map.yaml

运行前提：
    - slam_toolbox 正在运行（/map 有发布）
    - 已 source ROS 2 环境（/opt/ros2-foxy/install/setup.bash）
注意：
    /map 使用 transient_local（latched）QoS，本脚本已按匹配 QoS 订阅，
    即使地图发布很久后启动也能拿到最后一帧。
"""
import sys

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "map"

    rclpy.init()
    node = rclpy.create_node("map_saver")

    holder = {}
    qos = QoSProfile(
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(OccupancyGrid, "/map",
                             lambda m: holder.setdefault("m", m), qos)

    print("等待 /map ...（请确认 slam_toolbox 正在运行）")
    while rclpy.ok() and "m" not in holder:
        rclpy.spin_once(node, timeout_sec=0.5)

    if not rclpy.ok():
        return

    msg = holder["m"]
    w, h = msg.info.width, msg.info.height
    grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

    # OccupancyGrid: 0=free, 100=occupied, -1=unknown
    img = np.full((h, w), 205, dtype=np.uint8)   # unknown 灰
    img[grid == 0] = 254                          # free 白
    img[grid >= 65] = 0                           # occupied 黑
    img = np.flipud(img)                          # 原点在左下 → 图片坐标从左上

    with open(out + ".pgm", "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (w, h))
        f.write(img.tobytes())

    info = msg.info
    with open(out + ".yaml", "w") as f:
        f.write("image: %s.pgm\n" % out)
        f.write("resolution: %.6f\n" % info.resolution)
        f.write("origin: [%.6f, %.6f, 0.0]\n"
                % (info.origin.position.x, info.origin.position.y))
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.196\n")

    print("已保存: %s.pgm + %s.yaml  (%dx%d, %.3f m/px)"
          % (out, out, w, h, info.resolution))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
