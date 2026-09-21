#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
底盘串口探测 / 握手测试脚本  （RK3588 Toybrick + CH340G 转接）

用途：
  1. 列出本机全部串口设备（含 VID:PID）
  2. 自动扫描常见波特率，找出底盘驱动板能应答的那一个
  3. 在指定波特率下执行只读查询：转速 / 编码器 / 电池 / 温湿度
  4. 可选运动测试（PWM 前进 + 自动急停）

用法：
  python3 serial_probe.py                            # 仅列出串口
  python3 serial_probe.py -p /dev/ttyCH341USB0       # 扫描波特率
  python3 serial_probe.py -p /dev/ttyCH341USB0 -b 115200
  python3 serial_probe.py -p /dev/ttyCH341USB0 -b 115200 --move
  python3 serial_probe.py -p /dev/ttyCH341USB0 -b 115200 -l run1.log

安全须知：
  * AT+MTENCODER? 读取后计数值清零 —— 若 ROS 里程计节点正在运行，切勿执行本脚本。
  * --move 会真实驱动车轮：请先架空底盘或确保场地安全。脚本用 try/finally 保证
    无论出现任何异常，最后都会发送 AT+MT_STOP。
"""

import argparse
import glob
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial，请先执行： python3 -m pip install pyserial")
    sys.exit(1)

# 常见波特率，顺序按"最可能"排列，可被 -b 覆盖
COMMON_BAUDS = [115200, 9600, 57600, 38400, 19200, 230400, 460800, 921600, 256000, 128000]

_log_fh = None


def emit(*args):
    """同时输出到屏幕和日志文件"""
    text = " ".join(str(a) for a in args)
    print(text, flush=True)
    if _log_fh:
        _log_fh.write(text + "\n")
        _log_fh.flush()


def show_ports():
    rows = []
    seen = set()
    for p in list_ports.comports():
        vidpid = " [%04x:%04x]" % (p.vid, p.pid) if p.vid is not None else ""
        rows.append((p.device, (p.description or "") + vidpid))
        seen.add(p.device)

    # 兜底：直接扫 /dev。
    # 沁恒官方 usb_ch341 等非标准驱动不会出现在 pyserial 的枚举表里，需在此补上。
    fallback = []
    for pat in ("/dev/ttyCH341USB*", "/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyS*"):
        for dev in sorted(glob.glob(pat)):
            if dev not in seen:
                fallback.append(dev)

    if not rows and not fallback:
        emit("未发现任何串口设备（检查 USB 是否插好、ch341 模块是否加载）")
        return
    emit("检测到以下串口设备：")
    emit("-" * 62)
    for dev, desc in rows:
        emit("  %-22s %s" % (dev, desc))
    for dev in fallback:
        note = "板载 UART" if dev.startswith("/dev/ttyS") else "pyserial 未识别的驱动，可直接用 -p 指定"
        emit("  %-22s %s" % (dev, note))
    emit("-" * 62)
    emit("提示：CH340G 转接对应 /dev/ttyCH341USB0；板载 UART 为 /dev/ttyS3、/dev/ttyS7")


def transact(ser, cmd, wait=0.5, ending=b"\r\n", quiet=False):
    """发送一条 AT 指令并收集回应，返回解码后的字符串"""
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii", "ignore") + ending)
    ser.flush()
    if not quiet:
        emit("  >> %s" % cmd)
    buf = b""
    deadline = time.time() + wait
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            deadline = time.time() + 0.15   # 收到数据后略微延长，等待整帧结束
        else:
            time.sleep(0.02)
    # 温度字段含 GBK 编码的 ℃ 符号，UTF-8 解不出来时回退 gb18030
    try:
        text = buf.decode("utf-8")
    except UnicodeDecodeError:
        text = buf.decode("gb18030", "replace")
    text = text.strip()
    if not quiet:
        if text:
            for line in text.splitlines():
                emit("  << %s" % line)
        else:
            emit("  << (无回应)")
    return text


def probe_bauds(port, bauds, ending, timeout=0.35):
    """逐个波特率尝试握手，返回命中列表 [(baud, response), ...]"""
    hits = []
    emit("\n[1/2] 扫描波特率（使用只读指令 AT / AT+MTRPM?，不会驱动电机）")
    emit("-" * 62)
    ser = serial.Serial(port, baudrate=bauds[0], timeout=0.2)
    try:
        for b in bauds:
            try:
                ser.baudrate = b
            except Exception as e:
                emit("  [%7d] 无法设置该波特率: %s" % (b, e))
                continue
            time.sleep(0.15)
            resp = ""
            for cmd in ("AT", "AT+MTRPM?"):
                resp = transact(ser, cmd, wait=timeout, ending=ending, quiet=True)
                if resp:
                    break
            flat = resp.replace("\r", " ").replace("\n", " | ").strip()
            if resp:
                emit("  [%7d] 有回应: %s" % (b, flat[:90]))
                hits.append((b, resp))
            else:
                emit("  [%7d] 无回应" % b)
    finally:
        ser.close()
    emit("-" * 62)
    if hits:
        emit("命中波特率：%s" % ", ".join(str(b) for b, _ in hits))
    else:
        emit("全部无回应：请检查 TX/RX 是否交叉、GND 是否共地、底盘是否上电")
    return hits


def read_status(ser, ending):
    """只读状态查询（不发送任何运动指令）"""
    emit("\n[2/2] 只读状态查询")
    emit("-" * 62)
    for cmd in ("AT", "AT+MTRPM?", "AT+MTENCODER?", "AT+BATTERY?",
                "AT+TEMPERATURE?", "AT+HUMIDITY?"):
        transact(ser, cmd, wait=0.5, ending=ending)
    emit("-" * 62)
    emit("说明：AT+MTENCODER? 读取后计数值会被清零（协议规定），")
    emit("      如果之后要接里程计节点，注意从这一刻起的计数会重新从 0 开始。")


def move_test(ser, ending, pwm, duration):
    """运动测试：直行 duration 秒后急停。pwm 形如 '20,20,20,20'"""
    emit("\n[运动测试] 先发 AT+MT=ON,ON,ON,ON 使能，再发 AT+MT_SPWM=%s  持续 %.1f 秒，随后 AT+MT_STOP"
         % (pwm, duration))
    emit("!! 请确认底盘已架空或场地安全，人员远离车轮 !!")
    try:
        ans = input("确认继续请输入 yes 回车：").strip().lower()
    except EOFError:
        ans = ""
    if ans != "yes":
        emit("已取消运动测试。")
        return
    try:
        transact(ser, "AT+MT=ON,ON,ON,ON", wait=0.3, ending=ending)
        transact(ser, "AT+MT_SPWM=%s" % pwm, wait=0.3, ending=ending)
        time.sleep(duration)
    finally:
        transact(ser, "AT+MT_STOP", wait=0.5, ending=ending)
        emit("已发送 AT+MT_STOP（急停）")


def main():
    global _log_fh
    ap = argparse.ArgumentParser(
        description="底盘串口探测与握手测试（AT 协议）",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-p", "--port", help="串口设备，如 /dev/ttyCH341USB0")
    ap.add_argument("-b", "--baud", type=int, help="指定波特率；不加则自动扫描")
    ap.add_argument("-l", "--log", help="把输出同时写入日志文件")
    ap.add_argument("--move", action="store_true", help="附加运动测试（危险，需人工确认）")
    ap.add_argument("--pwm", default="-50,-50,-50,-50",
                    help="运动测试的 PWM 值（A,B,C,D，-100~100；负值=前进，默认 -50 前进）")
    ap.add_argument("--duration", type=float, default=1.0, help="运动持续时间秒（默认 1.0）")
    ap.add_argument("--ending", default="\\r\\n",
                    help="指令结尾，默认 \\r\\n；可改 \\r 或 ''（空）")
    args = ap.parse_args()

    if args.log:
        _log_fh = open(args.log, "w", encoding="utf-8")

    ending = args.ending.encode("utf-8").decode("unicode_escape").encode("utf-8")
    emit("=== 底盘串口探测 ===  时间：%s" % time.strftime("%Y-%m-%d %H:%M:%S"))
    emit("指令结尾：%r   超时：%d ms" % (ending, 500))

    if not args.port:
        show_ports()
        if not args.baud:
            emit("\n下一步：python3 serial_probe.py -p <上面列出的设备>")
        return

    # 无 -b 时自动扫描
    if not args.baud:
        hits = probe_bauds(args.port, COMMON_BAUDS, ending)
        if len(hits) == 1:
            args.baud = hits[0][0]
            emit("\n唯一命中，自动采用 %d 继续。" % args.baud)
        elif hits:
            emit("\n多个波特率有回应（可能是噪声/回显），请手动指定 -b 复测。")
            return
        else:
            return

    emit("\n使用波特率 %d 打开 %s ……" % (args.baud, args.port))
    try:
        ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.2)
    except Exception as e:
        emit("打开串口失败：%s" % e)
        return
    try:
        read_status(ser, ending)
        if args.move:
            move_test(ser, ending, args.pwm, args.duration)
        else:
            emit("\n提示：加 --move 可做运动测试（会先要求人工确认）。")
    finally:
        # 无论如何，关闭前确保电机停止
        try:
            ser.write(b"AT+MT_STOP" + ending)
            ser.flush()
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
    emit("\n完成。")


if __name__ == "__main__":
    main()
