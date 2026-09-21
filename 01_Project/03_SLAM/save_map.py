#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""保存 SLAM 地图（一条命令存两份，文件名自动带时间戳）

产出（同一前缀、配套一套；默认目录 `~/RobotCode/04_map/`）：

    map_YYYYmmdd_HHMMSS.pgm / .yaml         ← 栅格地图（看图 / nav2 map_server 用）
    map_YYYYmmdd_HHMMSS.posegraph / .data   ← 建图序列（可续建 / 定位模式加载）

用法：
    python3 save_map.py                  # 存到 ~/RobotCode/04_map/，名字 map_<时间戳>
    python3 save_map.py -n 房间A         # 自定义前缀（时间戳仍会拼在后面）
    python3 save_map.py -d /tmp/maps     # 换目录
    python3 save_map.py --no-pgm         # 只存建图序列（.posegraph + .data）
    python3 save_map.py --no-posegraph   # 只存栅格图（.pgm + .yaml）

前提：slam_toolbox 正在运行（`/map` 有发布、`slam_toolbox/serialize_map` 服务在线）。

说明：`save_pgm_yaml()` / `serialize_posegraph()` 也被 `phone_teleop.py` 的
「保存地图」按钮复用（手机上一键存图，走同一套逻辑）。
"""
import argparse
import os
import time

import numpy as np

try:
    import rclpy
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                           QoSReliabilityPolicy)
except ImportError as e:
    raise SystemExit(
        '导入 ROS 2 模块失败：%s\n提示：先 source 环境\n'
        '  source /opt/ros2-foxy/install/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

DEFAULT_SAVE_DIR = os.path.expanduser('~/RobotCode/04_map')


def save_pgm_yaml(msg, prefix):
    """把 OccupancyGrid 写成 <prefix>.pgm + <prefix>.yaml（nav2 map_server 兼容）

    yaml 里的 `image:` 写**相对文件名**（整个地图目录可以直接拷到别的机器用）。
    返回写出的文件路径列表。
    """
    w, h = msg.info.width, msg.info.height
    grid = np.array(msg.data, dtype=np.int16).reshape(h, w)

    img = np.full((h, w), 205, dtype=np.uint8)   # 未知 灰
    img[grid == 0] = 254                         # 空闲 白
    img[grid >= 65] = 0                          # 障碍 黑
    img = np.flipud(img)                         # 原点在左下 → 图片行从上到下

    pgm = prefix + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (w, h))
        f.write(img.tobytes())

    info = msg.info
    yml = prefix + '.yaml'
    with open(yml, 'w') as f:
        f.write('image: %s.pgm\n' % os.path.basename(prefix))
        f.write('resolution: %.6f\n' % info.resolution)
        f.write('origin: [%.6f, %.6f, 0.0]\n'
                % (info.origin.position.x, info.origin.position.y))
        f.write('negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return [pgm, yml]


def serialize_posegraph(node, prefix, timeout=30.0, spin=False):
    """调用 slam_toolbox 的 serialize_map 服务保存建图序列；返回 (ok, 说明)

    spin=True ：本线程 spin（命令行场景）
    spin=False：由外部 executor 处理（网页服务场景，rclpy 已在后台线程 spin）
    """
    from slam_toolbox.srv import SerializePoseGraph
    try:
        cli = node.create_client(SerializePoseGraph, 'slam_toolbox/serialize_map')
    except Exception as e:                     # 服务类型不可用（slam_toolbox 未构建/未 source）
        return False, '创建服务客户端失败：%s' % e
    if not cli.wait_for_service(timeout_sec=5.0):
        return False, '服务 slam_toolbox/serialize_map 不在（SLAM 没在运行？）'

    req = SerializePoseGraph.Request()
    req.filename = prefix
    fut = cli.call_async(req)
    t0 = time.time()
    while time.time() - t0 < timeout:
        if spin:
            rclpy.spin_once(node, timeout_sec=0.1)
        else:
            time.sleep(0.1)
        if fut.done():
            break

    if os.path.exists(prefix + '.posegraph'):
        return True, '已写出 %s.posegraph + .data' % os.path.basename(prefix)
    return False, '服务未在 %.0fs 内写出文件（超时或失败）' % timeout


def wait_map(node, timeout=10.0):
    """等 /map 第一帧（transient_local/latched，SLAM 在跑一般立刻拿到）"""
    holder = {}

    def cb(m):
        holder.setdefault('m', m)

    qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', cb, qos)
    t0 = time.time()
    while rclpy.ok() and 'm' not in holder and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    return holder.get('m')


def main():
    ap = argparse.ArgumentParser(
        description='保存 SLAM 地图（PGM+YAML 与建图序列，文件名自动带时间戳）')
    ap.add_argument('-d', '--dir', default=DEFAULT_SAVE_DIR,
                    help='保存目录（默认 ~/RobotCode/04_map）')
    ap.add_argument('-n', '--name', default='map',
                    help='文件名前缀（默认 map；时间戳会自动拼在后面）')
    ap.add_argument('--no-pgm', action='store_true', help='不存 PGM/YAML')
    ap.add_argument('--no-posegraph', action='store_true', help='不存建图序列')
    args = ap.parse_args()

    out_dir = os.path.expanduser(args.dir)
    prefix = os.path.join(out_dir, '%s_%s' % (args.name, time.strftime('%Y%m%d_%H%M%S')))
    try:
        os.makedirs(out_dir, exist_ok=True)
    except OSError as e:
        raise SystemExit('无法创建目录 %s：%s' % (out_dir, e))
    print('保存前缀：%s' % prefix)

    rclpy.init()
    node = rclpy.create_node('save_map')
    written = []
    try:
        if not args.no_pgm:
            m = wait_map(node, 10.0)
            if m is None:
                print('✗ 没收到 /map —— 确认 slam_toolbox 正在运行')
            else:
                written += save_pgm_yaml(m, prefix)
                print('✓ 栅格图  %s.pgm + .yaml（%dx%d @%.3f m/px）'
                      % (os.path.basename(prefix), m.info.width, m.info.height,
                         m.info.resolution))
        if not args.no_posegraph:
            ok, info = serialize_posegraph(node, prefix, spin=True)
            print('%s 建图序列 %s' % ('✓' if ok else '✗', info))
            if ok:
                written += [prefix + '.posegraph', prefix + '.data']
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    print('-' * 60)
    for p in written:
        if os.path.exists(p):
            print('  %-50s %9.1f KB' % (os.path.basename(p),
                                        os.path.getsize(p) / 1024.0))
    if not written:
        return 1
    print('完成。目录：%s' % out_dir)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
