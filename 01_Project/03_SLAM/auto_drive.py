#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自动行驶建图脚本 —— 用 /cmd_vel 让小车按预定路线自己走

用途：SLAM 建图（《04_SLAM建图实验方案.md》S5）时替代"人推车"——
速度恒定、直线更直、还能精确走回起点做闭环（loop closure）。

原理：订阅 /odom 做**闭环控制**（走了多远、转了多少以里程计读数为准，
不用时间估算），按 10~20 Hz 持续发布 /cmd_vel；一段到位即停，再走下一段。
速度按"临近目标减速 + 起步 0.5s 升速斜坡"处理，减少轮胎打滑、里程计更准。

前提：底盘节点（base_driver）在运行、`ros2 topic hz /odom` 有数据；建图时 SLAM 也在跑。

路线语法（-p 自定义，分号/空格分隔）：
    f1.5   前进 1.5 m           b0.5   后退 0.5 m
    t90    左转 90°（逆时针）   t-90   右转 90°
    s0.5   原地暂停 0.5 s       home   回到起点（转向→直行返回→转回原朝向）
    例：-p "f1;t90;f1;t-90;b1;home"

预设（直接给名字）：
    spin      原地转一圈（出发前让雷达扫全四周）
    straight  前进 D 再退回起点（直线往返）
    square    方形闭环：4 × ( 前进 D + 左转 90° )，终点≈起点
    grid      弓字形扫面：-L 行长、-W 行距、-n 行数
    home      从当前位置直接回起点

用法：
    python3 auto_drive.py spin                      # 原地转一圈
    python3 auto_drive.py square -d 1.5             # 1.5 m 方形闭环
    python3 auto_drive.py grid -L 2.0 -W 0.8 -n 3   # 扫 2m×2.4m 区域
    python3 auto_drive.py -p "f2;t90;f1;t90;home"   # 自定义路线
    python3 auto_drive.py square -d 1.5 --dry-run   # 只打印计划，不动车

参数：-v 前进速度 m/s（默认 0.15，建议 ≤0.2，雷达 6Hz 快了会糊）
      -w 转向角速度 rad/s（默认 0.4）

安全：
    * 全程 Ctrl-C 立即停车（底盘看门狗 0.5s 后也会自动停车）
    * /odom 断流 >1.5s（串口假死）自动中止并停车
    * 每段有超时保护（预计耗时 ×3 + 5s）
    * **不发横移速度 vy**：麦轮横移时差速里程计不可信（已知局限）
    * 无避障能力，请在清空场地、有人看护的情况下使用
"""
import argparse
import math
import re
import time

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
except ImportError as e:
    raise SystemExit(
        '导入 ROS 2 模块失败：%s\n提示：先 source 环境（或用 ~/.bash_aliases 里的别名）\n'
        '  source /opt/ros/humble/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

RATE = 20.0         # 控制/发布循环频率 Hz（须 > 看门狗 1/cmd_timeout）
ODOM_TIMEOUT = 1.5  # /odom 断流判据（秒）


class Abort(Exception):
    """需要立即停车退出的异常"""


def wrap(a):
    """角度归一化到 (-pi, pi]"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class AutoDriver(Node):
    """闭环速度控制：按距离/角度分段行驶，全程由 /odom 反馈判定到位"""

    def __init__(self, v_fwd, v_turn):
        super().__init__('auto_drive')
        self.v_fwd = v_fwd
        self.v_turn = v_turn
        self.pose = None            # (x, y, yaw) 当前里程计位姿
        self.last_odom = 0.0        # 最近一帧 /odom 到达时刻（本地时钟）
        self.start_pose = None      # 本轮起点（供 home 用）
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

    # ---------------- 基础 ----------------
    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, yaw)
        self.last_odom = time.time()

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=1.0 / RATE)

    def send(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

    def stop(self, secs=0.3):
        """持续发零速度（比只发一帧更稳妥）"""
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < secs:
            self.send(0.0, 0.0)
            self.spin_once()

    def check_alive(self):
        if time.time() - self.last_odom > ODOM_TIMEOUT:
            raise Abort('/odom 断流超过 %.1fs（底盘串口假死或节点退出？）'
                        % ODOM_TIMEOUT)

    def wait_odom(self, timeout=5.0):
        t0 = time.time()
        while self.pose is None and time.time() - t0 < timeout:
            self.spin_once()
        return self.pose is not None

    # ---------------- 两个基本动作 ----------------
    def drive(self, dist):
        """沿当前朝向直行 dist 米（负值=后退），闭环到 ±0.02 m。

        返回 (沿初始朝向的实际位移, 横向偏差)。
        """
        if abs(dist) < 1e-3:
            return 0.0, 0.0
        x0, y0, th0 = self.pose
        sign = 1.0 if dist > 0.0 else -1.0
        target = abs(dist)
        tol = 0.01
        t_start = time.time()
        timeout = target / self.v_fwd * 3.0 + 5.0
        prog = lat = 0.0

        while True:
            self.spin_once()
            self.check_alive()
            if time.time() - t_start > timeout:
                raise Abort('直行 %.2fm 超时（%.0fs，被挡住或打滑？）'
                            % (dist, timeout))
            x, y, th = self.pose
            dx, dy = x - x0, y - y0
            prog = dx * math.cos(th0) + dy * math.sin(th0)   # 沿初始朝向位移
            lat = -dx * math.sin(th0) + dy * math.cos(th0)   # 横向偏差
            err = target - prog
            if err <= tol:
                break
            v = min(self.v_fwd, max(0.04, 1.2 * err))        # 临近目标减速
            ramp = min(1.0, (time.time() - t_start) / 0.5)   # 起步升速斜坡
            self.send(sign * v * ramp, 0.0)

        self.stop(0.2)
        return prog, lat

    def turn(self, deg):
        """原地转 deg 度（正=左转/逆时针），闭环到 ±1.2°。返回实际转角。"""
        if abs(deg) < 0.1:
            return 0.0
        sign = 1.0 if deg > 0.0 else -1.0
        target = math.radians(abs(deg))
        tol = 0.01
        prev = self.pose[2]
        acc = 0.0                     # 累计转角（支持 >360°）
        t_start = time.time()
        timeout = target / self.v_turn * 3.0 + 5.0

        while True:
            self.spin_once()
            self.check_alive()
            if time.time() - t_start > timeout:
                raise Abort('转向 %.0f° 超时（%.0fs，轮子卡住或打滑？）'
                            % (deg, timeout))
            th = self.pose[2]
            acc += wrap(th - prev)
            prev = th
            err = target - abs(acc)
            if err <= tol:
                break
            # ⚠ P24 实测：末端减速到 ω=0.12 rad/s 会落进底盘死区（≈0.15）——最后 ~3°
            #   永远转不完，err 卡在 1.8° 直到超时（spin 真机实测：360° 只差 1.8° 卡住）。
            #   下限抬到 0.18：过冲 ≤0.5°/周期，1~2 个周期内收敛，且远高于死区。
            w = min(self.v_turn, max(0.18, 2.0 * err))
            ramp = min(1.0, (time.time() - t_start) / 0.4)
            self.send(0.0, sign * w * ramp)

        self.stop(0.2)
        return math.degrees(abs(acc))

    def homing(self):
        """回到起点：转向起点 → 直线返回 → 转回起始朝向"""
        x0, y0, th0 = self.start_pose
        x, y, th = self.pose
        d = math.hypot(x0 - x, y0 - y)
        if d < 0.05:
            turn_deg = math.degrees(wrap(th0 - th))
            self.turn(turn_deg)
            return d
        head = math.atan2(y0 - y, x0 - x)          # 世界系下朝起点的方向
        self.turn(math.degrees(wrap(head - th)))
        self.drive(d)
        x, y, th = self.pose
        self.turn(math.degrees(wrap(th0 - th)))    # 恢复起始朝向
        return d


# ---------------- 路线解析 / 预设 ----------------
def parse_route(text):
    segs = []
    for tok in re.split(r'[;\s]+', text.strip()):
        if not tok:
            continue
        key = tok[0].lower()
        if tok.lower() == 'home':
            segs.append(('home', 0.0))
            continue
        try:
            val = float(tok[1:])
        except ValueError:
            raise SystemExit('无法解析路线项：%s（示例 f1.5 / t90 / s0.5 / home）' % tok)
        if key == 'f':
            segs.append(('f', val))
        elif key == 'b':
            segs.append(('f', -val))
        elif key == 't':
            segs.append(('t', val))
        elif key == 's':
            segs.append(('s', val))
        else:
            raise SystemExit('未知路线符号：%s（支持 f / b / t / s / home）' % tok)
    return segs


def build_preset(name, dist, length, width, rows):
    if name == 'spin':
        return [('t', 360.0)]
    if name == 'straight':
        return [('f', dist), ('s', 0.5), ('f', -dist)]
    if name == 'square':
        segs = []
        for _ in range(4):
            segs += [('f', dist), ('t', 90.0)]
        return segs
    if name == 'grid':
        segs = []
        for i in range(rows):
            segs.append(('f', length))
            if i < rows - 1:
                sgn = 90.0 if i % 2 == 0 else -90.0
                segs += [('t', sgn), ('f', width), ('t', sgn)]
        return segs
    if name == 'home':
        return [('home', 0.0)]
    raise SystemExit('未知预设：%s' % name)


def describe(segs):
    lines = []
    for i, (key, val) in enumerate(segs, 1):
        if key == 'f':
            lines.append('%2d. 直行 %+.2f m' % (i, val))
        elif key == 't':
            lines.append('%2d. 原地转 %+.0f°' % (i, val))
        elif key == 's':
            lines.append('%2d. 暂停 %.2f s' % (i, val))
        else:
            lines.append('%2d. 回到起点（转向→直行返回→转回原朝向）' % i)
    return lines


def main():
    ap = argparse.ArgumentParser(
        description='自动行驶建图：按预设/自定义路线发 /cmd_vel，让小车自己走')
    ap.add_argument('preset', nargs='?', default=None,
                    choices=['spin', 'straight', 'square', 'grid', 'home'],
                    help='预设路线名（或用 -p 自定义）')
    ap.add_argument('-p', '--route', default=None,
                    help='自定义路线，如 "f2;t90;f1;t-90;home"')
    ap.add_argument('-d', '--distance', type=float, default=1.0,
                    help='straight/square 的行程 m（默认 1.0）')
    ap.add_argument('-L', '--length', type=float, default=2.0,
                    help='grid 的行长 m（默认 2.0）')
    ap.add_argument('-W', '--width', type=float, default=0.8,
                    help='grid 的行距 m（默认 0.8）')
    ap.add_argument('-n', '--rows', type=int, default=3,
                    help='grid 的行数（默认 3）')
    ap.add_argument('-v', '--speed', type=float, default=0.15,
                    help='前进速度 m/s（默认 0.15，建议 ≤0.2）')
    ap.add_argument('-w', '--turn-rate', type=float, default=0.4,
                    help='转向角速度 rad/s（默认 0.4）')
    ap.add_argument('--dry-run', action='store_true', help='只打印计划，不动车')
    ap.add_argument('--no-countdown', action='store_true', help='跳过 3 秒倒计时')
    args = ap.parse_args()

    if args.route:
        segs = parse_route(args.route)
    elif args.preset:
        segs = build_preset(args.preset, args.distance, args.length,
                            args.width, args.rows)
    else:
        ap.error('请给出预设名（spin/straight/square/grid/home）或用 -p 自定义路线')
    if not segs:
        ap.error('路线为空')

    # ---- 计划 ----
    print('=' * 56)
    print('自动行驶计划（%d 段）  前进 %.2f m/s | 转向 %.2f rad/s'
          % (len(segs), args.speed, args.turn_rate))
    print('=' * 56)
    for line in describe(segs):
        print('  ' + line)
    print('=' * 56)
    if args.dry_run:
        print('--dry-run：未发任何指令。')
        return 0

    rclpy.init()
    node = AutoDriver(args.speed, args.turn_rate)
    if not node.wait_odom(5.0):
        print('未收到 /odom —— 请先启动底盘节点（car），并用 odomhz 确认有数据。')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    node.start_pose = node.pose
    x0, y0, th0 = node.pose
    print('起点：x=%.3f  y=%.3f  yaw=%.1f°' % (x0, y0, math.degrees(th0)))

    if not args.no_countdown:
        print('3 秒后开始，随时 Ctrl-C 停车 ...')
        try:
            time.sleep(3.0)
        except KeyboardInterrupt:
            print('\n已取消。')
            node.destroy_node()
            rclpy.shutdown()
            return 0

    t_run = time.time()
    aborted = None
    try:
        for i, (key, val) in enumerate(segs, 1):
            if key == 'f':
                prog, lat = node.drive(val)
                print('[%d/%d] 直行 %+.2fm → 实测 %.2fm（横向偏差 %+.3fm）'
                      % (i, len(segs), val, prog, lat))
            elif key == 't':
                got = node.turn(val)
                print('[%d/%d] 转向 %+.0f° → 实测 %.1f°' % (i, len(segs), val, got))
            elif key == 's':
                print('[%d/%d] 暂停 %.1fs' % (i, len(segs), val))
                t0 = time.time()
                while time.time() - t0 < val:
                    node.spin_once()
                    node.send(0.0, 0.0)
            else:
                d = node.homing()
                print('[%d/%d] 回起点 → 返程 %.2fm' % (i, len(segs), d))
    except Abort as e:
        aborted = str(e)
        print('\n** 中止：%s **' % e)
    except KeyboardInterrupt:
        aborted = '用户中断（Ctrl-C）'
        print('\n** 用户中断 **')
    finally:
        node.stop(0.4)
        if node.pose is not None:
            x, y, th = node.pose
            print('-' * 56)
            print('终点相对起点：Δx=%+.3f m  Δy=%+.3f m  Δyaw=%+.1f°  用时 %.1fs'
                  % (x - x0, y - y0, math.degrees(wrap(th - th0)),
                     time.time() - t_run))
            print('起点 x=%.3f y=%.3f ｜ 终点 x=%.3f y=%.3f'
                  % (x0, y0, x, y))
        print('已停车。' + ('' if aborted is None else '（%s）' % aborted))
        print('提示：地图窗口 mapv ｜ 保存地图 savemap')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except rclpy._rclpy_pybind11.RCLError:
            pass    # Ctrl-C 时 rclpy 已自行 shutdown，重复调用会抛 RCLError，忽略即可
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
