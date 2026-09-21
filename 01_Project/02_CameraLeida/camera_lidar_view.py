#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机 + LD14P 雷达融合显示（OpenCV，不依赖 ROS）

画面：左半屏为相机实时画面，右半屏为雷达俯视图。
融合方式：把雷达正前方扇区（默认 ±31°，与相机水平视场角对应）的点
按角度映射到相机画面的列上，在图像底部画一条"距离地形带"：
  - 每列的高度 = 该方向最近障碍的相对远近（越近越高）
  - 颜色：>1.5m 绿，0.5~1.5m 黄，<0.5m 红
  - 白色竖线标记全画面最近的障碍方位，顶部显示其距离
当最近距离低于告警阈值（默认 0.5 m）时顶部显示红色 OBSTACLE 警告条。

前置条件：
  - 雷达：/dev/LD14P（udev 别名，或 -p 指定真实串口）
  - 相机：/dev/video0（-c 指定序号）
  - 依赖：cv2 / numpy / pyserial（本机已装）

用法：
  python3 camera_lidar_view.py                      # 有显示器时开窗
  python3 camera_lidar_view.py --headless -o f.jpg  # 无显示器：画面写文件
  python3 camera_lidar_view.py --fov 60 --warn 0.8  # 相机视场角/告警阈值

按键：q / Esc 退出。
"""

import argparse
import os
import time

import cv2
import numpy as np

from ld14p import Ld14p
from lidar_viewer import draw_radar, setup_display

WIN = "camera + lidar"
CAM_W, CAM_H = 640, 480


def build_column_ranges(ang, dis, fov, ncols):
    """把雷达前方扇区点按角度映射到相机每一列，返回每列最近距离(m)数组。"""
    col_min = np.full(ncols, np.inf, dtype=np.float32)
    if ang is None:
        return col_min
    # 只取前方 ±fov/2 扇区内的有效点
    in_sector = (dis > 0) & (ang >= -fov / 2) & (ang <= fov / 2)
    if not in_sector.any():
        return col_min
    cols = ((ang[in_sector] + fov / 2) / fov * (ncols - 1)).astype(int)
    ds = dis[in_sector]
    for c, d in zip(cols, ds):
        if d < col_min[c]:
            col_min[c] = d
    return col_min


def overlay_camera(frame, col_min, warn_dist, max_range):
    """在相机画面上叠加距离地形带和最近障碍标记，返回 (画面, 最近距离)"""
    h, w = frame.shape[:2]
    ncols = len(col_min)
    overlay = frame.copy()

    finite = np.isfinite(col_min)
    nearest = col_min[finite].min() if finite.any() else None

    # 底部距离地形带：列宽 = w/ncols，高度随距离减小而增大
    band_base = h - 1
    for c in range(ncols):
        if not np.isfinite(col_min[c]):
            continue
        d = col_min[c]
        x0 = int(c * w / ncols)
        x1 = int((c + 1) * w / ncols)
        height = int((1 - min(d / max_range, 1.0)) * h * 0.4)
        if d < warn_dist:
            color = (0, 0, 255)
        elif d < 1.5:
            color = (0, 255, 255)
        else:
            color = (0, 255, 0)
        cv2.rectangle(overlay, (x0, band_base - height), (x1, band_base),
                      color, -1)
    frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0)

    # 最近障碍方位白线 + 距离文字
    if nearest is not None:
        c = int(np.argmin(col_min))
        x = int((c + 0.5) * w / ncols)
        cv2.line(frame, (x, 0), (x, h), (255, 255, 255), 1)
        cv2.putText(frame, "%.2f m" % nearest, (min(x + 6, w - 110), 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        if nearest < warn_dist:
            cv2.rectangle(frame, (0, 0), (w, 34), (0, 0, 255), -1)
            cv2.putText(frame, "OBSTACLE %.2f m !" % nearest, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    return frame, nearest


def main():
    ap = argparse.ArgumentParser(description="相机 + LD14P 雷达融合显示",
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", default="/dev/LD14P", help="雷达串口")
    ap.add_argument("-c", "--camera", type=int, default=0, help="相机序号 /dev/videoN")
    ap.add_argument("--fov", type=float, default=62.0,
                    help="相机水平视场角(度)，决定取雷达前方多大扇区做映射")
    ap.add_argument("--warn", type=float, default=0.5, help="障碍告警阈值 m")
    ap.add_argument("--max-range", type=float, default=4.0,
                    help="距离归一化上限 m（超过按最远显示）")
    ap.add_argument("--headless", action="store_true", help="不开窗")
    ap.add_argument("-o", "--output", help="headless 下画面写入该图片文件")
    args = ap.parse_args()

    setup_display()
    headless = args.headless or not os.environ.get("DISPLAY")

    lidar = Ld14p(args.port)
    lidar.start()
    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        lidar.stop()
        raise SystemExit("打开相机 /dev/video%d 失败" % args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)
    print("雷达 %s + 相机 video%d 已启动（headless=%s），Ctrl+C 退出"
          % (args.port, args.camera, headless))

    last_stat = time.time()
    try:
        while True:
            ok, cam = cap.read()
            if not ok:
                print("相机读帧失败，重试中…")
                time.sleep(0.1)
                continue
            cam = cv2.resize(cam, (CAM_W, CAM_H))

            scan = lidar.get_scan()
            if scan:
                _, hz, ang_raw, dis, _ = scan
                # 右手系角度换成以正前方为 0 的带符号角：+左 -右
                ang = ((ang_raw + 180.0) % 360.0) - 180.0
                col_min = build_column_ranges(ang, dis, args.fov, 64)
            else:
                hz, col_min = 0.0, np.full(64, np.inf, np.float32)

            cam_view, nearest = overlay_camera(cam, col_min, args.warn,
                                               args.max_range)
            radar_view = draw_radar(scan, args.max_range, size=CAM_H)
            view = np.hstack((cam_view, radar_view))

            if headless:
                if args.output:
                    cv2.imwrite(args.output, view)
                if time.time() - last_stat >= 1.0:
                    last_stat = time.time()
                    msg = "组帧 %.1f Hz" % lidar.scan_rate()
                    if nearest is not None:
                        warn = " [警告!]" if nearest < args.warn else ""
                        msg += " | 前方最近 %.2f m%s" % (nearest, warn)
                    print(msg)
            else:
                cv2.imshow(WIN, view)
                if cv2.waitKey(30) & 0xFF in (27, ord("q")):
                    break
    except KeyboardInterrupt:
        print()
    finally:
        cap.release()
        lidar.stop()
        cv2.destroyAllWindows()
    print("已退出。")


if __name__ == "__main__":
    main()
