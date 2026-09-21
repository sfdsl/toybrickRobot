#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LD14P 雷达点云可视化（OpenCV 俯视图，不依赖 ROS）

画面说明（右手系，正前方朝上，左手边在左）：
  - 同心圆为距离环（默认每 1 m 一圈），顶部箭头为雷达正前方 0°
  - 点颜色按距离：近=红，远=绿；距离为 0 的无效点不画
  - 左上角 HUD：转速/组帧率/点数/最近障碍（距离+方位角）

用法：
  python3 lidar_viewer.py                          # 有显示器时直接开窗
  python3 lidar_viewer.py --headless               # 无显示器：终端打印统计
  python3 lidar_viewer.py --headless -o view.jpg   # 无显示器：同时把画面写到文件
  python3 lidar_viewer.py -p /dev/ttyACM0          # 指定串口（默认 /dev/LD14P）
  python3 lidar_viewer.py --max-range 6            # 量程刻度 6 m（默认 12）
  python3 lidar_viewer.py --dump 10 -o scan.csv    # 抓 10 圈存 CSV 后退出

按键：q / Esc 退出；+ / - 缩放量程。
"""

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

from ld14p import Ld14p

WIN = "LD14P scan"


def setup_display():
    """SSH / IDE 终端里没有 DISPLAY，尝试自动对接本机 Wayland 桌面的 Xwayland。

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


def polar_to_px(angle_deg, dist_m, cx, cy, scale):
    """右手系角度(正前方0, 逆时针+) -> 屏幕像素(前=上, 左=左)"""
    a = math.radians(angle_deg)
    return int(cx - dist_m * scale * math.sin(a)), \
           int(cy - dist_m * scale * math.cos(a))


def draw_radar(scan, max_range, size=640):
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
        cv2.putText(img, "waiting scan...", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return img

    stamp, hz, ang, dis, inten = scan

    # 画点：近红远绿
    for a, d in zip(ang, dis):
        if d <= 0 or d > max_range:
            continue
        px, py = polar_to_px(a, d, cx, cy, scale)
        ratio = min(d / max_range, 1.0)
        color = (0, int(255 * ratio), int(255 * (1 - ratio)))
        cv2.circle(img, (px, py), 2, color, -1)

    # 最近障碍
    valid = dis > 0
    hud = ["speed %.2f Hz | points %d" % (hz, len(ang))]
    if valid.any():
        i = np.argmin(np.where(valid, dis, np.inf))
        a = ang[i] - 360.0 if ang[i] > 180.0 else ang[i]   # 显示为 -180~180
        hud.append("nearest %.2f m @ %.1f deg" % (dis[i], a))
    for k, line in enumerate(hud):
        cv2.putText(img, line, (10, 25 + 22 * k),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
    return img


def dump_csv(path, scans):
    """把若干圈点云写成 CSV：frame,angle_deg,dist_m,intensity"""
    with open(path, "w") as f:
        f.write("frame,angle_deg,dist_m,intensity\n")
        for idx, (stamp, hz, ang, dis, inten) in enumerate(scans):
            for a, d, it in zip(ang, dis, inten):
                f.write("%d,%.3f,%.4f,%d\n" % (idx, a, d, int(it)))
    print("已保存 %d 圈 -> %s" % (len(scans), path))


def main():
    ap = argparse.ArgumentParser(description="LD14P 雷达点云可视化",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/LD14P", help="串口设备")
    ap.add_argument("--max-range", type=float, default=12.0, help="量程刻度 m")
    ap.add_argument("--headless", action="store_true",
                    help="不开窗（无 DISPLAY 时自动启用）")
    ap.add_argument("-o", "--output", help="headless 下把画面持续写入该图片文件")
    ap.add_argument("--dump", type=int, metavar="N", help="抓 N 圈存 CSV 后退出")
    args = ap.parse_args()

    setup_display()
    headless = args.headless or not os.environ.get("DISPLAY")
    lidar = Ld14p(args.port)
    lidar.start()
    print("雷达 %s 已启动（headless=%s），Ctrl+C 退出" % (args.port, headless))

    max_range = args.max_range
    scans = []
    last_stat = time.time()
    try:
        while True:
            scan = lidar.get_scan()
            if args.dump:
                if scan and (not scans or scan[0] != scans[-1][0]):
                    scans.append(scan)
                    print("已采集 %d/%d 圈" % (len(scans), args.dump), end="\r")
                    if len(scans) >= args.dump:
                        break
                time.sleep(0.01)
                continue

            img = draw_radar(scan, max_range)
            if headless:
                if args.output:
                    cv2.imwrite(args.output, img)
                if time.time() - last_stat >= 1.0:
                    last_stat = time.time()
                    if scan:
                        _, hz, ang, dis, _ = scan
                        valid = dis > 0
                        if valid.any():
                            i = np.argmin(np.where(valid, dis, np.inf))
                            a = ang[i] - 360.0 if ang[i] > 180.0 else ang[i]
                            print("组帧 %.1f Hz | %d 点 | 最近 %.2f m @ %.1f°"
                                  % (lidar.scan_rate(), len(ang), dis[i], a))
                        else:
                            print("组帧 %.1f Hz | %d 点 | 无有效回波"
                                  % (lidar.scan_rate(), len(ang)))
            else:
                cv2.imshow(WIN, img)
                key = cv2.waitKey(30) & 0xFF
                if key in (27, ord("q")):
                    break
                if key in (ord("+"), ord("=")):
                    max_range = max(1.0, max_range - 1.0)
                if key == ord("-"):
                    max_range = min(12.0, max_range + 1.0)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print()
    finally:
        lidar.stop()
        cv2.destroyAllWindows()
    if args.dump:
        dump_csv(args.output or "scan.csv", scans)
    print("已退出。")


if __name__ == "__main__":
    main()
