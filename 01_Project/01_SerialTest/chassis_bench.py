#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘交互式台架工具 —— 键盘控制 + 编码器/转速/电池实时反馈

用途：
  1. 验证 4 路 PWM（A,B,C,D）与车轮的实际对应关系（前进 / 后退 / 转向方向标定）
  2. 观察编码器读数是否随车轮转动而变化（里程计的前置验证）
  3. 脱离 ROS 快速试底盘手感

用法：
  python3 chassis_bench.py -p /dev/ttyCH341USB0
  python3 chassis_bench.py -p /dev/ttyCH341USB0 --pwm 30
  python3 chassis_bench.py -p /dev/ttyCH341USB0 --left "-30,30,-30,30"

按键：
  w 前进    s 后退    a 左转    d 右转
  空格 急停（AT+MT_STOP）
  + / -     调整 PWM 幅度（调整时会先急停）
  r 刷新电池    q 退出（退出自动急停）

安全提示：
  * 本工具独占串口：运行期间不要同时开 ROS 节点或其它串口程序
  * 编码器查询会清零计数（协议规定），本工具内部自行累加并显示累计值
  * 首次使用请架空底盘；--fwd/--back/--left/--right 的默认值只是常见猜测，
    请按实测结果用参数调整
"""

import argparse
import select
import sys
import termios
import time
import tty

import serial


def parse_four(text, name):
    try:
        vals = [int(x) for x in str(text).split(",")]
    except ValueError:
        raise argparse.ArgumentTypeError("%s 需要形如 20,20,20,20 的 4 个整数" % name)
    if len(vals) != 4:
        raise argparse.ArgumentTypeError("%s 需要 4 个整数" % name)
    return vals


def scale4(pwm4, mag):
    """保持各通道符号，把幅度缩放到 mag"""
    return [mag if v > 0 else (-mag if v < 0 else 0) for v in pwm4]


class Chassis(object):
    def __init__(self, ser, ending):
        self.ser = ser
        self.ending = ending
        self.rpm = (0, 0)
        self.enc = [0, 0]
        self.batt = "--"
        self.pwm = (0, 0, 0, 0)
        self.enabled = False

    # ---------------- 底层收发 ----------------
    def send(self, text, wait=0.12):
        """发送 AT 指令并收集回应（0.12s 足够覆盖一次查询往返）"""
        self.ser.reset_input_buffer()
        self.ser.write(text.encode("ascii", "ignore") + self.ending)
        self.ser.flush()
        buf = b""
        deadline = time.time() + wait
        while time.time() < deadline:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
                deadline = time.time() + 0.05
            else:
                time.sleep(0.01)
        try:
            return buf.decode("utf-8")
        except UnicodeDecodeError:
            return buf.decode("gb18030", "replace")

    def query_data(self, name, wait=0.12):
        """查询 AT+NAME?，返回剔除 +NAME: 与 OK 行后的数据串"""
        text = self.send("AT+%s?" % name, wait=wait)
        out = []
        for line in text.splitlines():
            line = line.strip()
            if not line or line == "OK" or line.startswith("+"):
                continue
            out.append(line)
        return " ".join(out)

    # ---------------- 业务动作 ----------------
    def drive(self, pwm4):
        self.pwm = tuple(pwm4)
        self.send("AT+MT_SPWM=%d,%d,%d,%d" % self.pwm, wait=0.05)

    def stop(self):
        self.pwm = (0, 0, 0, 0)
        self.send("AT+MT_STOP", wait=0.1)

    def enable(self):
        """电机使能。例程确认：不发这条，SPWM 指令虽返回 OK 但车轮不动"""
        self.send("AT+MT=ON,ON,ON,ON", wait=0.1)

    def poll(self):
        """读转速与编码器增量并累加"""
        r = self.query_data("MTRPM")
        if r:
            parts = r.replace(",", " ").split()
            if len(parts) >= 2:
                try:
                    self.rpm = (int(parts[0]), int(parts[1]))
                except ValueError:
                    pass
        e = self.query_data("MTENCODER")
        if e:
            parts = e.replace(",", " ").split()
            if len(parts) >= 2:
                try:
                    self.enc[0] += int(parts[0])
                    self.enc[1] += int(parts[1])
                except ValueError:
                    pass

    def read_battery(self):
        b = self.query_data("BATTERY", wait=0.2)
        if b:
            self.batt = b.split(",")[0]

    def render(self, tag):
        sys.stdout.write(
            "\rPWM %-18s RPM %5d/%-5d  ENC %9d/%-9d  BAT %-8s [%s]      "
            % (",".join(str(v) for v in self.pwm), self.rpm[0], self.rpm[1],
               self.enc[0], self.enc[1], self.batt, tag))
        sys.stdout.flush()


def run_interactive(ch, args, p, fwd, back, left, right):
    fd = sys.stdin.fileno()
    old_term = termios.tcgetattr(fd)
    tag = "停止"
    last_poll = 0.0
    last_batt = 0.0
    try:
        tty.setcbreak(fd)
        print("=== 底盘台架工具 ===")
        print("按键： w 前进 | s 后退 | a 左转 | d 右转 | 空格 急停 | +/- 调速 | r 电池 | q 退出")
        print("映射： 前进 %s | 后退 %s | 左转 %s | 右转 %s（PWM 幅度 %d%%）"
              % (fwd, back, left, right, p))
        print(        "-" * 78)
        ch.stop()
        ch.enable()
        ch.read_battery()
        while True:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist:
                key = sys.stdin.read(1)
                if key in ("q", "Q", "\x03"):
                    tag = "退出"
                    break
                elif key in ("w", "W"):
                    ch.drive(fwd)
                    tag = "前进"
                elif key in ("s", "S"):
                    ch.drive(back)
                    tag = "后退"
                elif key in ("a", "A"):
                    ch.drive(left)
                    tag = "左转"
                elif key in ("d", "D"):
                    ch.drive(right)
                    tag = "右转"
                elif key == " ":
                    ch.stop()
                    tag = "急停"
                elif key in ("+", "="):
                    p = min(100, p + 5)
                    fwd, back, left, right = (scale4(fwd, p), scale4(back, p),
                                              scale4(left, p), scale4(right, p))
                    ch.stop()
                    tag = "调速到 %d%%" % p
                elif key in ("-", "_"):
                    p = max(5, p - 5)
                    fwd, back, left, right = (scale4(fwd, p), scale4(back, p),
                                              scale4(left, p), scale4(right, p))
                    ch.stop()
                    tag = "调速到 %d%%" % p
                elif key in ("r", "R"):
                    ch.read_battery()
                    tag = "已刷新电池"
            now = time.time()
            if now - last_poll >= args.interval:
                last_poll = now
                ch.poll()
                ch.render(tag)
            if now - last_batt >= 3.0:
                last_batt = now
                ch.read_battery()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
        try:
            ch.stop()
        except Exception:
            pass
    print("\n已发送 AT+MT_STOP。")


def main():
    ap = argparse.ArgumentParser(description="底盘交互式台架工具（键盘控制 + 实时反馈）")
    ap.add_argument("-p", "--port", default="/dev/ttyCH341USB0", help="串口设备")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="波特率（实测为 115200）")
    ap.add_argument("--pwm", type=int, default=50, help="默认 PWM 幅度 1~100（默认 50，例程用 40~80）")
    ap.add_argument("--fwd", type=lambda s: parse_four(s, "--fwd"), default=None,
                    help="前进的 4 通道值，默认 -PWM 全部（负值=前进）")
    ap.add_argument("--back", type=lambda s: parse_four(s, "--back"), default=None,
                    help="后退的 4 通道值，默认 +PWM 全部")
    ap.add_argument("--left", type=lambda s: parse_four(s, "--left"), default=None,
                    help="左转的 4 通道值，默认 +PWM,-PWM,+PWM,-PWM")
    ap.add_argument("--right", type=lambda s: parse_four(s, "--right"), default=None,
                    help="右转的 4 通道值，默认 -PWM,+PWM,-PWM,+PWM")
    ap.add_argument("--interval", type=float, default=0.2, help="状态刷新间隔秒（默认 0.2）")
    ap.add_argument("--ending", default="\\r\\n", help="指令结尾（默认 \\r\\n）")
    args = ap.parse_args()

    if not sys.stdin.isatty():
        print("本工具为键盘交互模式，请在真实终端里运行。")
        return 1

    ending = args.ending.encode("utf-8").decode("unicode_escape").encode("utf-8")
    p = max(1, min(100, args.pwm))
    # 方向/通道定义取自配套例程 Car_Control.h：
    #   负值 = 前进、正值 = 后退；A/C = 左侧轮，B/D = 右侧轮
    fwd = args.fwd or [-p, -p, -p, -p]
    back = args.back or [p, p, p, p]
    left = args.left or [p, -p, p, -p]
    right = args.right or [-p, p, -p, p]

    try:
        ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.05)
    except Exception as e:
        print("打开串口失败：%s" % e)
        return 1
    try:
        ch = Chassis(ser, ending)
        run_interactive(ch, args, p, fwd, back, left, right)
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
