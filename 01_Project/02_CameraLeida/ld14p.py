#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LD14P 激光雷达纯 Python 驱动（不依赖 ROS）

协议依据：《LD14P激光雷达开发手册 v0.1》第 4 章
  - 串口参数：230400 / 8N1 / 无流控
  - 数据包固定 47 字节：
      0      起始符   0x54
      1      VerLen   0x2C (高3位帧类型=1, 低5位点数=12)
      2-3    雷达转速  u16 LE, 单位 度/秒
      4-5    起始角度  u16 LE, 单位 0.01 度
      6-41   12 个测量点, 每点 3 字节: 距离 u16 LE(mm) + 信号强度 u8
      42-43  结束角度  u16 LE, 单位 0.01 度
      44-45  时间戳    u16 LE, 单位 ms (0~30000 循环)
      46     CRC8 校验 (对前 46 字节, 查表法, 表见手册)
  - 点角度 = 起始角度 + i * (结束角度-起始角度)/11  (线性插值)

角度修正依据：官方 SDK sl_transform.cpp Transform()
  三角测距的测距中心与旋转中心不重合，LD14P 修正参数：
      offset_tan = 0.11923, offset_x = 5.9, offset_y = -18.975571
  注意：LD14P 只修正角度，距离保持原始值（与 LD00/03/08 不同）。

坐标系约定：
  雷达原始数据为左手系：正前方 0 度，顺时针增大。
  to_right_hand=True（默认）时输出右手系（ROS 约定）：正前方 0 度，逆时针增大。

典型用法：
    from ld14p import Ld14p
    lidar = Ld14p("/dev/LD14P")
    lidar.start()
    scan = lidar.get_scan()      # None 或 (stamp, speed_hz, angles, dists, intens)
    lidar.stop()

注意：同一时间只能有一个进程占用串口，运行本驱动前请先停掉
      `ros2 launch ldlidar ld14p.launch.py`。
"""

import threading
import time
from collections import deque

import numpy as np

try:
    import serial
except ImportError:
    raise SystemExit("缺少 pyserial，请先执行： python3 -m pip install pyserial")

# ---------------- 协议常量（手册 4.1 节） ----------------
PKG_HEADER = 0x54
PKG_VER_LEN = 0x2C            # 帧类型1 + 12个点
POINT_PER_PACK = 12
PACK_SIZE = 47                # 整包长度
POINT_FREQ = 4000             # LD14P 测距频率 4000 次/秒
DEFAULT_BAUD = 230400

# LD14P 三角测距修正参数（SDK sl_transform.cpp）
OFFSET_TAN = 0.11923
OFFSET_X = 5.9
OFFSET_Y = -18.975571

# CRC8 查表（手册 4.1 节，与官方 SDK lipkg.cpp 一致）
_CRC_TABLE = (
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8,
)


def calc_crc8(data):
    """对 data 计算 CRC8（查表法），返回 1 字节校验值"""
    crc = 0
    for b in data:
        crc = _CRC_TABLE[(crc ^ b) & 0xFF]
    return crc


def parse_packet(buf, to_right_hand=True):
    """
    解析一个 47 字节完整数据包（不含 CRC 校验，校验由调用方完成）。
    返回 (speed_dps, [(angle_deg, dist_m, intensity), ...])，失败返回 None。
    """
    speed = buf[2] | (buf[3] << 8)                 # 度/秒
    start_raw = buf[4] | (buf[5] << 8)             # 0.01 度
    end_raw = buf[42] | (buf[43] << 8)

    # 跨 360 度处理 + 合理性过滤（SDK lipkg.cpp 的过滤条件）
    diff_raw = (end_raw + 36000 - start_raw) % 36000
    if speed == 0:
        return None
    if diff_raw / 100.0 > (speed * POINT_PER_PACK / POINT_FREQ * 1.5):
        return None

    step = diff_raw / (POINT_PER_PACK - 1) / 100.0
    start = start_raw / 100.0
    pts = []
    for i in range(POINT_PER_PACK):
        off = 6 + i * 3
        dist_mm = buf[off] | (buf[off + 1] << 8)
        inten = buf[off + 2]
        angle = (start + i * step) % 360.0
        pts.append((angle, dist_mm / 1000.0, inten))
    return speed, pts


def transform_points(raw_pts, to_right_hand=True):
    """
    三角测距角度修正 + 坐标系转换（对应 SDK sl_transform.cpp，LD14P 分支）。
    LD14P 只修角度，距离不变。
    """
    out = []
    last_shift = 0.0
    for angle, dist, inten in raw_pts:
        if dist > 0:
            x = dist * 1000.0 + OFFSET_X
            y = dist * 1000.0 * OFFSET_TAN + OFFSET_Y
            shift = np.degrees(np.arctan(y / x))
            last_shift = shift
        else:
            shift = last_shift
        if to_right_hand:
            a = (360.0 - angle) + shift
        else:
            a = angle - shift
        a %= 360.0
        out.append((a, dist, inten if dist > 0 else 0))
    return out


class Ld14p:
    """LD14P 串口驱动：后台线程读串口、组帧，主线程取最新一圈点云。"""

    def __init__(self, port="/dev/LD14P", baud=DEFAULT_BAUD, to_right_hand=True):
        self.port = port
        self.baud = baud
        self.to_right_hand = to_right_hand

        self._ser = None
        self._thread = None
        self._running = False
        self._lock = threading.Lock()

        self._raw_pts = []          # 当前正在积累的一圈（未修正）
        self._last_frame = None     # 最近一整圈 (stamp, speed_hz, angles, dists, intens)
        self._frame_times = deque(maxlen=30)

        # 统计信息
        self.pkts_ok = 0
        self.pkts_crc_err = 0
        self.bytes_in = 0

    # ---------------- 生命周期 ----------------
    def start(self):
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            raise SystemExit(
                "打开 %s 失败：%s\n"
                "排查：1) 雷达是否插好  2) 是否有别的进程占用（如 ros2 launch）\n"
                "      3) 权限：ls -l %s （toybrick 需在 dialout 组）"
                % (self.port, e, self.port))
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._ser and self._ser.is_open:
            self._ser.close()

    # ---------------- 数据获取 ----------------
    def get_scan(self):
        """取最近一整圈点云。
        返回 None 或 (stamp, speed_hz, angles_deg, dists_m, intensities)，
        后三项为 numpy 数组，已按角度升序排序。"""
        with self._lock:
            return self._last_frame

    def scan_rate(self):
        """最近约 5 秒内的组帧频率（Hz）。"""
        now = time.time()
        cnt = sum(1 for t in self._frame_times if now - t < 5.0)
        return cnt / 5.0

    # ---------------- 内部：读串口 + 解包 + 组帧 ----------------
    def _read_loop(self):
        buf = bytearray()
        while self._running:
            try:
                n = self._ser.in_waiting
                chunk = self._ser.read(n if n > 0 else 1)
            except serial.SerialException:
                break
            if not chunk:
                continue
            self.bytes_in += len(chunk)
            buf += chunk
            buf = self._consume(buf)

    def _consume(self, buf):
        """从缓冲区中找完整数据包并解析，返回剩余未处理字节。"""
        while True:
            idx = buf.find(PKG_HEADER)
            if idx < 0:
                return bytearray()
            if idx > 0:
                del buf[:idx]
            if len(buf) < 2:
                return buf
            if buf[1] != PKG_VER_LEN:
                del buf[0]
                continue
            if len(buf) < PACK_SIZE:
                return buf
            if calc_crc8(buf[:PACK_SIZE - 1]) == buf[PACK_SIZE - 1]:
                self.pkts_ok += 1
                parsed = parse_packet(buf, self.to_right_hand)
                if parsed:
                    self._accumulate(*parsed)
                del buf[:PACK_SIZE]
            else:
                self.pkts_crc_err += 1
                del buf[0]

    def _accumulate(self, speed_dps, pts):
        """积累点并在角度回绕时完成一整圈。"""
        last_angle = self._raw_pts[-1][0] if self._raw_pts else 0.0
        for angle, dist, inten in pts:
            # 一圈结束判定：角度从 >340 跨到 <20（SDK lipkg.cpp 组帧逻辑）
            if angle < 20.0 and last_angle > 340.0 and len(self._raw_pts) > 100:
                self._finish_frame(speed_dps)
                self._raw_pts = []
            self._raw_pts.append((angle, dist, inten))
            last_angle = angle

    def _finish_frame(self, speed_dps):
        corrected = transform_points(self._raw_pts, self.to_right_hand)
        corrected.sort(key=lambda p: p[0])
        arr = np.asarray(corrected, dtype=np.float32)
        with self._lock:
            self._last_frame = (time.time(), speed_dps / 360.0,
                                arr[:, 0], arr[:, 1], arr[:, 2])
        self._frame_times.append(time.time())


if __name__ == "__main__":
    # 简单自测：打印 5 秒的统计信息
    lidar = Ld14p()
    lidar.start()
    print("LD14P 自检：%s @ %d" % (lidar.port, lidar.baud))
    t0 = time.time()
    while time.time() - t0 < 5:
        time.sleep(1.0)
        scan = lidar.get_scan()
        if scan:
            stamp, hz, ang, dis, inten = scan
            valid = dis[dis > 0]
            print("scan: %d 点, 转速 %.2f Hz, 组帧 %.1f Hz, "
                  "有效点 %d, 最近 %.3f m @ %.1f°, CRC错误 %d"
                  % (len(ang), hz, lidar.scan_rate(), len(valid),
                     valid.min() if len(valid) else 0,
                     ang[dis.argmin()] if len(valid) else 0,
                     lidar.pkts_crc_err))
        else:
            print("暂无完整一圈数据…（包 ok=%d crc_err=%d）"
                  % (lidar.pkts_ok, lidar.pkts_crc_err))
    lidar.stop()
    print("自检完成。")
