#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
base_driver 底盘节点：/cmd_vel ⇄ 串口 ⇄ /odom + TF

    /cmd_vel ──► 麦轮逆运动学 ──► AT+MT_SPWM=A,B,C,D ──► 驱动板
    /odom   ◄── 差速积分      ◄── AT+MTENCODER?(20Hz) ◄── 驱动板

协议实测结论（详见 02_doc/01_环境问题清单.md P1-3 / P1-4）：
  * 115200 8N1，指令结尾 CR+LF
  * 必须先使能：AT+MT=ON,ON,ON,ON（漏发则 SPWM 只被接收、车轮不动）
  * PWM 符号：负值 = 车轮前进、正值 = 后退（与直觉相反）
  * 通道分组：A、C = 左侧轮；B、D = 右侧轮
  * 台架实测标定（2026-09-17，chassis_bench.py）：
      前进 -x,-x,-x,-x    后退 +x,+x,+x,+x
      左转 +x,-x,+x,-x    右转 -x,+x,-x,+x
      左移 +x,-x,-x,+x    右移 -x,+x,+x,-x
  * 编码器仅 A、B 两路且"读取即清零"（增量式），本节点做差速积分。
    已知局限：麦轮横移时左右两轮反向，差速模型会把侧移误算为自转；
    后续若能从驱动板拿到 4 路编码器再升级为麦轮里程计。

线程模型：必须单线程（rclpy.spin 默认即单线程）。
  串口为独占资源、一次只能完成一条指令的往返，所有读写必须串行。

常用参数（ros2 run base_driver chassis_node --ros-args -p 名称:=值）：
  port              串口设备，默认 /dev/ttyCH341USB0
  baudrate          波特率，默认 115200
  poll_rate         编码器轮询频率 Hz，默认 20
  cmd_timeout       /cmd_vel 看门狗超时 秒，超时自动停车，默认 0.5
  vx_max / vy_max / wz_max   速度映射：对应 PWM 100 的速度值
  ticks_per_meter   里程标定：每米编码器脉冲数（默认 3645，2026-09-18 地面多次标定）
  wheel_separation  等效转向参数（默认 0.4085，2026-09-18 转圈标定，含转向滑移修正；
                    理论几何 ≈ 轴距+轮距 = 0.155+0.202 = 0.357；非物理轮距，物理轮距实测 0.2023）
  enc_left_sign / enc_right_sign  编码器方向（前进时读数为正则填 1，否则 -1）
  publish_tf        是否广播 odom→base_link，默认 True
  dry_run           True 时不打开串口（无硬件验证 ROS 图），默认 False
  debug_enc         True 时以 1Hz 打印编码器增量（标定用），默认 False
"""

import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

import serial

# 指令结尾 CR + LF（协议要求，等价于常见的 "\r\n"）
CRLF = bytes((13, 10))


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ChassisSerial(object):
    """AT 协议串口层（与 chassis_bench.py 相同的收发逻辑，已台架验证）"""

    def __init__(self, port, baud):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, text, wait=0.1, quiet=0.015):
        """发送一条指令并收集响应，返回解码后的文本。

        正常路径：响应以 OK 结尾即立即返回（约 5~15ms）；
        兜底：连续 quiet 秒无新数据、或总等待超过 wait 秒即返回。
        """
        self.ser.reset_input_buffer()
        self.ser.write(text.encode("ascii", "ignore") + CRLF)
        self.ser.flush()
        buf = b""
        t0 = time.time()
        last_rx = t0
        while True:
            n = self.ser.in_waiting
            if n:
                buf += self.ser.read(n)
                last_rx = time.time()
                if buf.rstrip().endswith(b"OK"):
                    break
            else:
                now = time.time()
                if now - last_rx > quiet or now - t0 > wait:
                    break
                time.sleep(0.002)
        try:
            return buf.decode("utf-8")
        except UnicodeDecodeError:
            return buf.decode("gb18030", "replace")

    def query(self, name, wait=0.1):
        """AT+NAME? -> 返回剔除回显 / '+NAME:' / 'OK' 行后的数据串"""
        text = self.send("AT+%s?" % name, wait=wait)
        out = []
        for line in text.splitlines():
            line = line.strip()
            if not line or line == "OK" or line.startswith("+") or line.startswith("AT+"):
                continue
            out.append(line)
        return " ".join(out)

    def spwm(self, pwm4):
        self.send("AT+MT_SPWM=%d,%d,%d,%d" % tuple(pwm4), wait=0.05)

    def enable(self):
        self.send("AT+MT=ON,ON,ON,ON", wait=0.1)

    def stop(self):
        self.send("AT+MT_STOP", wait=0.1)


class ChassisNode(Node):
    """底盘节点：订阅 /cmd_vel 驱动车轮，轮询编码器发布 /odom + TF"""

    def __init__(self):
        super().__init__('base_driver')

        # ---------------- 参数 ----------------
        self.declare_parameter('port', '/dev/ttyCH341USB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('poll_rate', 20.0)
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('vx_max', 0.5)
        self.declare_parameter('vy_max', 0.4)
        self.declare_parameter('wz_max', 2.0)
        self.declare_parameter('ticks_per_meter', 3645.0)
        self.declare_parameter('wheel_separation', 0.4085)
        self.declare_parameter('enc_left_sign', 1)
        self.declare_parameter('enc_right_sign', 1)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('debug_enc', False)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baudrate').value)
        self.poll_rate = float(self.get_parameter('poll_rate').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.vx_max = float(self.get_parameter('vx_max').value)
        self.vy_max = float(self.get_parameter('vy_max').value)
        self.wz_max = float(self.get_parameter('wz_max').value)
        self.ticks_per_meter = float(self.get_parameter('ticks_per_meter').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.enc_left_sign = int(self.get_parameter('enc_left_sign').value)
        self.enc_right_sign = int(self.get_parameter('enc_right_sign').value)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.debug_enc = bool(self.get_parameter('debug_enc').value)

        self.odom_frame = 'odom'
        self.base_frame = 'base_link'

        # ---------------- 状态 ----------------
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_cmd_time = time.time()
        self.cmd_active = False
        self.last_pwm = None
        self.poll_count = 0
        self.enc_total = [0, 0]
        self.last_odom_time = self.get_clock().now()

        # ---------------- 串口 ----------------
        if self.dry_run:
            self.ser = None
            self.get_logger().warn('dry_run 模式：不打开串口，仅打印 PWM')
        else:
            try:
                self.ser = ChassisSerial(self.port, self.baud)
            except Exception as e:
                raise RuntimeError(
                    '打开串口失败：%s\n'
                    '提示：确认 %s 存在、当前用户已在 dialout 组，'
                    '且未被其它程序占用（chassis_bench / 串口助手等）。' % (e, self.port))
            self.ser.stop()    # 清掉上电残留的运动指令
            self.ser.enable()  # 使能电机（不发这条 SPWM 无效）
            self.get_logger().info('串口已打开：%s @ %d，电机已使能' % (self.port, self.baud))

        # ---------------- ROS 接口 ----------------
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_timer(1.0 / self.poll_rate, self.on_poll)

        self.get_logger().info(
            'base_driver 就绪：轮询 %.0fHz | cmd 超时 %.2fs | ticks_per_meter=%.1f '
            'wheel_separation=%.3fm（未标定前 /odom 只反映变化趋势，数值不可信）'
            % (self.poll_rate, self.cmd_timeout, self.ticks_per_meter, self.wheel_separation))

    # ---------------- /cmd_vel ----------------
    def on_cmd_vel(self, msg):
        self.last_cmd_time = time.time()
        self.cmd_active = True
        self.send_pwm(self.twist_to_pwm(msg.linear.x, msg.linear.y, msg.angular.z))

    def twist_to_pwm(self, vx, vy, wz):
        """麦轮逆运动学（映射已按 2026-09-17 台架实测标定）。

        实测对照（PWM 负值 = 车轮前进），通道 A(左前) B(右前) C(左后) D(右后)：
          前进  -x,-x,-x,-x    后退 +x,+x,+x,+x
          左转  +x,-x,+x,-x    右转 -x,+x,-x,+x
          左移  +x,-x,-x,+x    右移 -x,+x,+x,-x
        """
        ax = clamp(100.0 * vx / self.vx_max, -100.0, 100.0)
        ay = clamp(100.0 * vy / self.vy_max, -100.0, 100.0)
        aw = clamp(100.0 * wz / self.wz_max, -100.0, 100.0)

        a = -ax + ay + aw
        b = -ax - ay - aw
        c = -ax - ay + aw
        d = -ax + ay - aw
        return [int(round(clamp(v, -100.0, 100.0))) for v in (a, b, c, d)]

    def send_pwm(self, pwm4):
        if pwm4 == self.last_pwm:
            return  # 值未变化不重发，把串口带宽留给编码器轮询
        self.last_pwm = pwm4
        if self.ser is None:
            self.get_logger().info('dry_run PWM: %s' % pwm4, throttle_duration_sec=1.0)
            return
        try:
            self.ser.spwm(pwm4)
        except Exception as e:
            self.get_logger().error('SPWM 发送失败：%s' % e, throttle_duration_sec=2.0)

    # ---------------- 编码器轮询 + 里程计 ----------------
    def on_poll(self):
        now = time.time()

        # 看门狗：/cmd_vel 超时未更新 -> 柔和停车（保持使能，便于重新加速）
        if self.cmd_active and (now - self.last_cmd_time) > self.cmd_timeout:
            self.cmd_active = False
            self.get_logger().warn(
                'cmd_vel 超时 %.2fs，自动停车' % self.cmd_timeout, throttle_duration_sec=2.0)
            self.send_pwm([0, 0, 0, 0])

        if self.ser is None:
            return

        # 读编码器（读取即清零，返回自上次读取以来的增量）
        try:
            data = self.ser.query('MTENCODER')
        except Exception as e:
            self.get_logger().error('编码器读取失败：%s' % e, throttle_duration_sec=2.0)
            return

        vals = self._parse_ints(data)
        if len(vals) < 2:
            return

        self.poll_count += 1
        self.enc_total[0] += vals[0]
        self.enc_total[1] += vals[1]
        if self.debug_enc and self.poll_count % max(1, int(self.poll_rate)) == 0:
            self.get_logger().info(
                'ENC 增量/累计：A %d/%d  B %d/%d'
                % (vals[0], self.enc_total[0], vals[1], self.enc_total[1]))

        # 差速里程计积分（编码器 A = 左侧轮、B = 右侧轮）
        d_left = vals[0] * self.enc_left_sign / self.ticks_per_meter
        d_right = vals[1] * self.enc_right_sign / self.ticks_per_meter
        dc = 0.5 * (d_left + d_right)
        dtheta = (d_right - d_left) / self.wheel_separation

        self.x += dc * math.cos(self.theta + 0.5 * dtheta)
        self.y += dc * math.sin(self.theta + 0.5 * dtheta)
        self.theta = wrap_angle(self.theta + dtheta)

        ros_now = self.get_clock().now()
        dt = (ros_now - self.last_odom_time).nanoseconds * 1e-9
        self.last_odom_time = ros_now
        if dt <= 1e-6:
            dt = 1.0 / self.poll_rate

        self.publish_odom(ros_now.to_msg(), dc / dt, dtheta / dt)

    @staticmethod
    def _parse_ints(text):
        out = []
        for tok in text.replace(',', ' ').split():
            try:
                out.append(int(tok))
            except ValueError:
                pass
        return out

    def publish_odom(self, stamp, vx, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.theta * 0.5)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        # 粗略协方差：编码器里程计 x/y/yaw 相对可信，其余轴不估计
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is None:
            return
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.theta * 0.5)
        t.transform.rotation.w = math.cos(self.theta * 0.5)
        self.tf_broadcaster.sendTransform(t)

    # ---------------- 退出 ----------------
    def shutdown(self):
        if self.ser is not None:
            try:
                self.ser.stop()
            except Exception:
                pass
            self.ser.close()
            self.get_logger().info('已发送 AT+MT_STOP 并关闭串口')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ChassisNode()
    except RuntimeError as e:
        print(e)
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    main()
