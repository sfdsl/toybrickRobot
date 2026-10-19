#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""电源板 IMU（ICM-42670）→ ROS 2 节点，发布 /imu/data

对应《02_doc/05_实施清单/卡-02-EKF融合-实施清单.md》步骤 1。
板载六轴挂在 **I2C-6 @ 0x69**（WHO_AM_I = 0x67），与《01_环境问题清单.md》记录一致。

三种用法：
    python3 imu_driver.py                  # 正常跑：发布 /imu/data（默认 100 Hz）
    python3 imu_driver.py --check          # 自检：打印原始值与换算值（不进 ROS）
    python3 imu_driver.py --calibrate 60   # 零偏标定：静止采 60s，输出零偏与噪声

硬件事实（2026-10-15 实测，本文件据此编写）：
    * 厂商示例 sd_power/imu_sample 的换算系数**正确**，与实测吻合：
        加速度 16384 LSB/g   （即 ±2g 量程）
        陀螺   131.072 LSB/(°/s)（即 ±250 dps 量程）
        温度   raw/128 + 25 ℃
    * 初始化序列完全照搬 sd_power/imu/imu.c 的 imu_init()（含 bank1 的 0x1001 写入）
    * **字节序是小端**：基址寄存器放低字节（0x09/0x0A = TEMP 低/高）。
      厂商 C 示例能算对是因为它把 2 字节直接 *(int16_t*) 强转，小端机上恰好等价；
      Python 里必须显式 '<h'，否则温度会变成 −33℃ 这种离谱值。
    * 数据寄存器 **0x09~0x16 连续**（TEMP + 加速度 + 陀螺），一次 14 字节连读即可：
      实测 461 Hz，逐个寄存器分别读只有 94 Hz（快 5 倍）。
    * 陀螺**上电后 ~0.2 s 才出数**（此前读到的恒为 0），init() 末尾等 0.3 s。
    * 静止平放时 **accel_z ≈ -1 g** → 该电源板装好后 IMU 的 **+z 轴朝下**
      （z 是竖直轴，说明板面是水平装的）。朝向/位置标定见清单步骤 2.2。
    * 2026-10-15 静止 60 s 标定结果：
        陀螺零偏 (-0.00833, -0.00732, +0.00501) rad/s
        陀螺噪声 std (0.0206, 0.0046, 0.0081) rad/s ← 下方协方差默认值 = std² × 2 余量

参数：
    --rate      发布频率 Hz（默认 100；IMU 实际 ODR 1kHz，100Hz 轮询有混叠，够用）
    --frame     发布用的 frame_id（默认 imu_link）
    --i2c       I2C 设备（默认 /dev/i2c-6）
    --gyro-cov  角速度方差（协方差对角线，默认 0.01，标定后按 噪声std² 填）
    --accel-cov 加速度方差（默认 0.1）
    --gyro-bias 陀螺零偏 [x,y,z]（rad/s，标定后填）

注意（卡-02 业务规则）：
    * 协方差**绝不能是 0** —— 0 表示"完全可信"，EKF 会独信 IMU。
    * orientation 不发布（本 IMU 无姿态解算），orientation_covariance[0] = -1 表示无效。
"""
import argparse
import fcntl
import math
import os
import struct
import sys
import time

I2C_SLAVE = 0x0703                  # linux/i2c-dev.h
IMU_ADDR = 0x69
IMU_WHOAMI = 0x67

# 数据寄存器（bank 0，8 位地址；**低字节在前 → 小端 int16**，见 read_raw 的说明）
REG_TEMP = 0x09
REG_ACC_X = 0x0B
REG_ACC_Y = 0x0D
REG_ACC_Z = 0x0F
REG_GYR_X = 0x11
REG_GYR_Y = 0x13
REG_GYR_Z = 0x15

# 灵敏度（由 imu_init() 的 0x20/0x21 <- 0x66 决定：FS 位 = 11）
ACC_LSB_PER_G = 16384.0             # ±2g
GYR_LSB_PER_DPS = 131.072           # ±250 dps
G = 9.80665
ACC_SCALE = G / ACC_LSB_PER_G                    # raw → m/s²
GYR_SCALE = (math.pi / 180.0) / GYR_LSB_PER_DPS  # raw → rad/s


class ICM42670(object):
    """ICM-42670 的 I2C 访问（读写序列与 sd_power/imu/imu.c 保持一致）"""

    def __init__(self, dev='/dev/i2c-6', addr=IMU_ADDR):
        self.dev = dev
        self.addr = addr
        self.fd = None

    # ---------- 底层 ----------
    def open(self):
        self.fd = os.open(self.dev, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, self.addr)
        return self

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None

    def _write(self, buf):
        if os.write(self.fd, bytes(buf)) != len(buf):
            raise IOError('I2C 写入失败：%s' % self.dev)

    def _read(self, n):
        data = b''
        while len(data) < n:            # os.read 可能短读，补足
            chunk = os.read(self.fd, n - len(data))
            if not chunk:
                raise IOError('I2C 读取失败：%s' % self.dev)
            data += chunk
        return data

    def w8(self, reg, val):
        self._write([reg & 0xFF, val & 0xFF])

    def r8(self, reg):
        self._write([reg & 0xFF])
        return self._read(1)[0]

    def r16(self, reg):
        """读 2 字节 → 有符号 **小端** int16

        实测（2026-10-15）：基址寄存器放的是**低字节**（0x0B = ACC_X[7:0]、0x0C = ACC_X[15:8]）。
        厂商 C 示例能算对，是因为它把 2 字节直接 *(int16_t*) 强转，在小端机上恰好等价；
        Python 里必须显式按小端解包，否则数值完全错乱（表现为温度变成 −33℃）。
        """
        self._write([reg & 0xFF])
        return struct.unpack('<h', self._read(2))[0]

    # ---------- 时钟/存储体（照搬 C 驱动的 mclk 逻辑） ----------
    def _mclk_on(self):
        buf = self.r8(0x1F)
        if not (buf & 0x10):
            self.w8(0x1F, buf | 0x10)
            for _ in range(100000):
                if self.r8(0x00) & 0x08:
                    return
            raise IOError('IMU MCLK 开启超时')

    def _mclk_off(self):
        buf = self.r8(0x1F)
        if buf & 0x10:
            self.w8(0x1F, buf & ~0x10)

    def w16(self, reg, val):
        """写 bank1 寄存器（reg 形如 0x10xx），内部自动开/关 MCLK"""
        blk = (reg & 0xFF00) >> 8
        if blk == 0x10:
            blk = 0x00
        self._mclk_on()
        if blk:
            self.w8(0x79, blk)
        self.w8(0x7A, reg & 0x00FF)
        self.w8(0x7B, val & 0xFF)
        if blk:
            self.w8(0x79, 0x00)
        self._mclk_off()

    # ---------- 初始化（序列与 imu_init() 逐条对应） ----------
    def init(self):
        who = self.r8(0x75)
        if who != IMU_WHOAMI:
            raise IOError('WHO_AM_I 不符：读到 0x%02X，期望 0x%02X（@%s 0x%02X）'
                          % (who, IMU_WHOAMI, self.dev, self.addr))
        self.w8(0x36, 0x41)
        self.w8(0x35, 0xB0)
        time.sleep(0.003)
        self.w8(0x02, 0x10)
        self.w8(0x36, 0x41)
        self.w8(0x35, 0xB0)
        time.sleep(0.001)
        buf = self.r8(0x35)
        self.w8(0x35, buf & ~0x10)
        if not (self.r8(0x3A) & 0x10):
            raise IOError('IMU 软复位未完成')
        self.w8(0x21, 0x66)     # ACCEL_CONFIG0：±2g / ODR 1kHz
        self.w8(0x20, 0x66)     # GYRO_CONFIG0 ：±250dps / ODR 1kHz
        self.w8(0x22, 0x30)
        self.w8(0x06, 0x03)
        self.w16(0x1001, 0x00)
        buf = self.r8(0x1F)
        self.w8(0x1F, buf | 0x8F)   # 加速度计 + 陀螺仪 进入低噪声模式
        # 实测：上电后陀螺要 ~0.2 s 才出数，之前读到的一律是 0。等 0.3 s 留余量
        time.sleep(0.3)
        return True

    # ---------- 读数 ----------
    def read_raw(self):
        """一次 14 字节连读 0x09~0x16（TEMP + ACC + GYR，寄存器连续）

        实测 461 Hz；逐寄存器分别读只有 94 Hz。每对字节都是 [低, 高] → 小端。
        """
        self._write([REG_TEMP])
        t, ax, ay, az, gx, gy, gz = struct.unpack('<7h', self._read(14))
        return {'acc': (ax, ay, az), 'gyr': (gx, gy, gz), 'temp': t}

    def read_si(self, bias=(0.0, 0.0, 0.0)):
        raw = self.read_raw()
        acc = tuple(v * ACC_SCALE for v in raw['acc'])
        gyr = tuple(raw['gyr'][i] * GYR_SCALE - bias[i] for i in range(3))
        return acc, gyr, raw['temp'] / 128.0 + 25.0, raw


# ---------------------------------------------------------------- 自检
def do_check(dev, addr, n=6):
    imu = ICM42670(dev, addr).open()
    try:
        imu.init()
        print('WHO_AM_I = 0x%02X ✓ 初始化完成\n' % IMU_WHOAMI)
        for i in range(n):
            acc, gyr, temp, raw = imu.read_si()
            mod = math.sqrt(sum(v * v for v in acc)) / G
            print('[%d] 原始 acc=%s gyr=%s | 换算 a=(%6.2f,%6.2f,%6.2f) m/s²  '
                  'w=(%7.4f,%7.4f,%7.4f) rad/s | 模长 %.3f g | %.1f ℃'
                  % (i, raw['acc'], raw['gyr'], acc[0], acc[1], acc[2],
                     gyr[0], gyr[1], gyr[2], mod, temp))
            time.sleep(0.2)
        acc, _, _, _ = imu.read_si()
        vertical = ['x', 'y', 'z'][max(range(3), key=lambda k: abs(acc[k]))]
        print('\n竖直轴 = %s（|读数| 最大）；静止平放时该轴应 ≈ ±9.81 m/s²，'
              '符号为负说明该轴正方向朝下' % vertical)
        print('accel 模长应 ≈ 9.81 m/s²（1g）')
    finally:
        imu.close()


# ---------------------------------------------------------------- 零偏标定
def do_calibrate(dev, addr, seconds, rate):
    imu = ICM42670(dev, addr).open()
    try:
        imu.init()
        print('静止平放、别碰车，采集 %d 秒 ...' % seconds)
        acc_s, gyr_s, raw_s = [], [], []
        dt = 1.0 / rate
        t0 = time.time()
        n = 0
        while time.time() - t0 < seconds:
            raw = imu.read_raw()
            acc_s.append([v * ACC_SCALE for v in raw['acc']])
            gyr_s.append([v * GYR_SCALE for v in raw['gyr']])
            raw_s.append(list(raw['gyr']))
            n += 1
            time.sleep(dt)
    finally:
        imu.close()

    if n < 10:
        print('样本太少（%d），标定失败' % n)
        return 1

    def stat(cols):
        out = []
        for c in cols:
            m = sum(c) / len(c)
            var = sum((v - m) ** 2 for v in c) / len(c)
            out.append((m, math.sqrt(var)))
        return out

    am = stat([[s[i] for s in acc_s] for i in range(3)])
    gm = stat([[s[i] for s in gyr_s] for i in range(3)])
    print('\n采集 %d 点 / %.1f s\n' % (n, seconds))
    print('== 加速度（m/s²）==')
    for i, k in enumerate('xyz'):
        print('  %s: 零偏 %+8.4f   噪声 std %.4f' % (k, am[i][0], am[i][1]))
    print('== 陀螺（rad/s）==')
    for i, k in enumerate('xyz'):
        print('  %s: 零偏 %+8.5f   噪声 std %.5f' % (k, gm[i][0], gm[i][1]))
    print('\n竖直轴（|零偏|≈9.81 的那个）的零偏**不要**减掉，那是重力：')
    print('  扣除重力后的加速度零偏 = 竖直轴零偏减去 ±9.81（符号与轴朝向有关）')

    print('\n---- 复制到 ekf.launch.py / imu_driver 启动参数 ----')
    print('gyro_bias: %.6f,%.6f,%.6f' % (gm[0][0], gm[1][0], gm[2][0]))
    print('（就是上面这一整串，逗号分隔、不含空格，直接填 ekf.launch.py 的 gyro_bias）')
    print('# 卡-02 业务规则：协方差 = 方差 = std²。下面是 IMU 角速度的实测方差：')
    for i, k in enumerate('xyz'):
        print('  %s 轴角速度方差 = %.6f' % (k, gm[i][1] ** 2))
    return 0


# ---------------------------------------------------------------- ROS 发布
def do_publish(args):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = rclpy.create_node('imu_driver')
    log = node.get_logger()

    imu = ICM42670(args.i2c, args.addr).open()
    try:
        imu.init()
        log.info('IMU 就绪：%s @0x%02X，WHO_AM_I=0x%02X，发布 /imu/data %.0f Hz'
                 % (args.i2c, args.addr, IMU_WHOAMI, args.rate))
    except Exception as e:
        log.error('IMU 初始化失败：%s' % e)
        imu.close()
        rclpy.shutdown()
        return 1

    bias = tuple(args.gyro_bias)
    pub = node.create_publisher(Imu, 'imu/data', 20)
    period = 1.0 / args.rate
    last = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            now = time.time()
            if now - last < period:
                time.sleep(0.0005)
                continue
            last = now
            try:
                acc, gyr, temp, _ = imu.read_si(bias)
            except Exception as e:
                log.warn('I2C 读失败：%s（IMU 掉线？）' % e, throttle_duration_sec=2.0)
                continue

            msg = Imu()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = args.frame
            # 姿态：本节点不做姿态解算（卡-02 §5 也只融合角速度）→ 标记为不可用
            msg.orientation_covariance[0] = -1.0
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gyr
            for i in range(3):      # 逐轴方差：三轴噪声差很多（x 轴最差），不能一个值糊过去
                msg.angular_velocity_covariance[i * 4] = args.gyro_cov[i]
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = acc
            for i in range(3):
                msg.linear_acceleration_covariance[i * 4] = args.accel_cov[i]
            pub.publish(msg)
    except KeyboardInterrupt:
        pass
    finally:
        imu.close()
        rclpy.shutdown()
    return 0


def vec3(text):
    """解析 "x,y,z"（也容忍空格分隔）—— 供 launch 传参用

    launch 的 ExecuteProcess 把每个 cmd 元素原样当**一个** argv 传下去，
    所以不能靠 nargs=3 让 launch 传三个数（会变成带空格的单个 token）；
    统一用逗号分隔的单个 token 最稳。
    """
    parts = text.replace(',', ' ').split()
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('需要 3 个数，如 -0.0083,-0.0073,0.0050（收到 %r）' % text)
    try:
        return [float(p) for p in parts]
    except ValueError:
        raise argparse.ArgumentTypeError('不是合法数字：%r' % text)


def do_yaw_test():
    """朝向判定（清单步骤 2.2）：手推车原地转 ~180°，读**累计转角**

    比"盯着 ros2 topic echo 找正负"靠谱得多：噪声 ±0.015 rad/s 积分 3 秒才 ±0.045 rad，
    而转半圈是 π rad —— 信噪比几十倍，一眼就能定，不用在噪声里找符号。

    前提：别处已经在发 /imu/data（即 bringup/launch 已起来）。
    """
    import rclpy
    from sensor_msgs.msg import Imu

    rclpy.init()
    node = rclpy.create_node('imu_yaw_test')
    st = {'sum': 0.0, 'last': None}

    def cb(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if st['last'] is not None:
            dt = t - st['last']
            if 0.0 < dt < 0.5:                     # 丢包/跳变的时间间隔不积分
                st['sum'] += msg.angular_velocity.z * dt
        st['last'] = t

    node.create_subscription(Imu, '/imu/data', cb, 50)
    print('''
＝＝ IMU 朝向判定（清单步骤 2.2）＝＝
  1) 先别动车，等 3 秒 —— 累计转角应该基本不动（验证零偏已扣）
  2) 手推车原地转约 180°（半圈），2~3 秒内转完
  3) 停手，再等 2 秒，然后 Ctrl-C
  记住你转的方向：从上往下看是逆时针还是顺时针。
''')
    t0 = time.time()
    next_print = 0.0
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.5)
            now = time.time() - t0
            # spin_once 在 100 Hz 消息下会立刻返回，不节流就是每秒上百行
            if now >= next_print:
                next_print = now + 0.5
                print('  t=%5.1fs   累计转角 = %+7.3f rad  =  %+7.1f°'
                      % (now, st['sum'], math.degrees(st['sum'])))
    except KeyboardInterrupt:
        pass
    except Exception:                  # 被 Ctrl-\ / kill 中断时也要把结论打出来
        pass
    print('\n完成：累计转角 = %+.1f°' % math.degrees(st['sum']))
    print('''
判读（用最后那个数，别用中间过程）：
  你转的是【逆时针 / 左转】约 180°：
      读 +180° → IMU 的 z 与 base_link 同向 → imu_roll 保持 0.0
      读 -180° → z 轴反了                → imu_roll 改 3.14159
  你转的是【顺时针 / 右转】约 180°：上面两条对调

  判定后把值填进 robot_bringup/launch/ekf.launch.py 的 DEFAULT_IMU_RPY，
  以后启动就不用再手打 imu_roll 了。
''')
    try:
        node.destroy_node()
        rclpy.shutdown()
    except Exception:      # 已被外部中断（Ctrl-\、kill）时上下文可能已关
        pass


def main(argv=None):
    ap = argparse.ArgumentParser(description='电源板 IMU（ICM-42670）→ /imu/data')
    ap.add_argument('--check', action='store_true', help='自检：打印原始值与换算值')
    ap.add_argument('--yaw-test', action='store_true',
                    help='朝向判定：手推车转 180°，打印累计转角（需别处已发 /imu/data）')
    ap.add_argument('--calibrate', type=float, default=0, metavar='SEC',
                    help='零偏标定：静止采集 SEC 秒')
    ap.add_argument('--i2c', default='/dev/i2c-6', help='I2C 设备（默认 /dev/i2c-6）')
    ap.add_argument('--addr', type=lambda s: int(s, 0), default=IMU_ADDR, help='I2C 地址')
    ap.add_argument('--rate', type=float, default=100.0, help='发布频率 Hz（默认 100）')
    ap.add_argument('--frame', default='imu_link', help='frame_id（默认 imu_link）')
    ap.add_argument('--gyro-cov', type=float, nargs=3,
                    default=[8.5e-4, 4.3e-5, 1.3e-4], metavar=('X', 'Y', 'Z'),
                    help='角速度方差（std²），默认取 2026-10-15 实测噪声 ×2 余量；'
                         '卡-02 只融合 z 轴，z 是默认 1.3e-4')
    ap.add_argument('--accel-cov', type=float, nargs=3,
                    default=[1.0, 0.12, 0.06], metavar=('X', 'Y', 'Z'),
                    help='加速度方差（std²），默认取实测噪声 ×2 余量')
    ap.add_argument('--gyro-bias', type=vec3, default=[0.0, 0.0, 0.0],
                    metavar='X,Y,Z',
                    help='陀螺零偏 rad/s，逗号分隔（标定后填，例 -0.008333,-0.007322,0.005007）')
    args = ap.parse_args(argv)

    if args.check:
        return do_check(args.i2c, args.addr)
    if args.yaw_test:
        return do_yaw_test()
    if args.calibrate > 0:
        return do_calibrate(args.i2c, args.addr, args.calibrate, args.rate)
    return do_publish(args)


if __name__ == '__main__':
    sys.exit(main())
