#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ekf_check —— 卡-02 验收取证工具：并排对比 /odom（轮速）与 /odometry/filtered（EKF）

为什么需要它：
    auto_drive.py 是**用 /odom 做反馈**的闭环控制，所以
        drive(5)   → "走到 /odom 自己说 5 m 就停"
        turn(360)  → "转到 /odom 自己说 360° 就停"
    停下时 /odom 必然报残差≈0 —— **循环论证**，测的是控制精度，不是里程计精度。
    要判断里程计准不准，必须另外拿一个源来比。本工具同时订阅两个源，
    给出各自相对起点的位移，以及两者之差。

用法：
    python3 ekf_check.py              # 一直记录，Ctrl-C 出总结
    python3 ekf_check.py -d 60        # 记 60 秒后自动出总结（静止漂移用）
    python3 ekf_check.py -r 2         # 每 2 秒打一行（默认 1 秒）

对应验收项（卡-02 §6.1 / §6.2）：
    验收 1  直线 5 m 横向偏移 < 3 cm  → 看总结里 /odometry/filtered 的 **Δy**
    验收 2  原地 360° 累计误差 < 5°   → 看两行的 Δyaw
                                       （真值要另用 imu_driver.py --yaw-test 积分陀螺）
    验收 5  EKF 是否滞后于轮速       → 看"运动中 yaw 差 max"与"折算滞后 ms"；
                                       末段残差≈0 表示只是滞后、无偏置
    §6.2    零速度静止 60 s 位姿漂移  → -d 60，看 Δx/Δy/Δyaw 是否≈0

两个踩过的坑（已在本文件里规避）：
  * yaw 差**必须 wrap**：不 wrap 的话，在 ±180° 回绕那一刻会算出 358° 这种假值
  * Ctrl-C 后 rclpy 上下文可能已被关，再 shutdown 会抛 RCLError → 包 try
"""
import argparse
import math
import sys
import time

try:
    import rclpy
    from nav_msgs.msg import Odometry
except ImportError as e:
    raise SystemExit(
        '导入 ROS 2 模块失败：%s\n提示：先 source 环境（或用 ~/.bash_aliases 里的别名）\n'
        '  source /opt/ros/humble/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

TOPIC_ODOM = '/odom'
TOPIC_EKF = '/odometry/filtered'


def yaw_of(q):
    """四元数 → yaw（rad）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    """把角度折到 (−π, π]。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def pose_of(msg):
    """Odometry → (x, y, yaw)。"""
    return (msg.pose.pose.position.x, msg.pose.pose.position.y,
            yaw_of(msg.pose.pose.orientation))


def main():
    ap = argparse.ArgumentParser(
        description='并排对比 /odom 与 /odometry/filtered（卡-02 验收取证）')
    ap.add_argument('-d', '--duration', type=float, default=0.0,
                    help='记录秒数，到点自动出总结（默认 0 = 一直记，Ctrl-C 结束）')
    ap.add_argument('-r', '--rate', type=float, default=1.0,
                    help='打印周期秒（默认 1.0）')
    ap.add_argument('--wait', type=float, default=10.0,
                    help='等待两个话题出现的超时秒（默认 10）')
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node('ekf_check')
    st = {'odom': None, 'ekf': None}
    node.create_subscription(Odometry, TOPIC_ODOM,
                             lambda m: st.__setitem__('odom', m), 50)
    node.create_subscription(Odometry, TOPIC_EKF,
                             lambda m: st.__setitem__('ekf', m), 50)

    print('等待话题 %s 与 %s ...' % (TOPIC_ODOM, TOPIC_EKF))
    t0 = time.time()
    while ((st['odom'] is None or st['ekf'] is None)
           and time.time() - t0 < args.wait):
        rclpy.spin_once(node, timeout_sec=0.1)

    if st['odom'] is None or st['ekf'] is None:
        miss = [t for t, v in ((TOPIC_ODOM, st['odom']),
                               (TOPIC_EKF, st['ekf'])) if v is None]
        print('超时：没有收到 %s' % ' 和 '.join(miss))
        if len(miss) == 2:
            print('提示：栈没起来？→ ros2 launch robot_bringup bringup.launch.py '
                  'phone:=false ekf:=true')
        elif st['ekf'] is None:
            print('提示：EKF 没在跑 → 启动时加 ekf:=true')
        else:
            print('提示：底盘没上电 / 没发 /odom？')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        return 1

    o0, e0 = pose_of(st['odom']), pose_of(st['ekf'])
    print('起点  /odom=(%.3f, %.3f, %+.2f°)   EKF=(%.3f, %.3f, %+.2f°)'
          % (o0[0], o0[1], math.degrees(o0[2]),
             e0[0], e0[1], math.degrees(e0[2])))
    print('开始记录（Ctrl-C 结束）\n')

    t_start = time.time()
    nxt = 0.0
    prev_yaw, prev_t = None, None
    max_diff = 0.0          # 运动中 |odom−EKF| 的最大值（度）
    lags = []               # 折算滞后（毫秒）
    diffs = []              # 全部采样 |差分|（度），用于看末段残差

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.time()
            el = now - t_start
            if args.duration > 0.0 and el >= args.duration:
                break

            o, e = pose_of(st['odom']), pose_of(st['ekf'])
            d = math.degrees(wrap(o[2] - e[2]))          # ← 必须 wrap

            # 用 /odom 的角速度把"角度差"折算成"时间差"，比只看角度直观
            rate = 0.0
            if prev_yaw is not None and prev_t is not None:
                dt = now - prev_t
                if dt > 1e-3:
                    rate = wrap(o[2] - prev_yaw) / dt    # rad/s
            prev_yaw, prev_t = o[2], now
            if abs(rate) > 0.10:                         # 只在明显转动时统计
                max_diff = max(max_diff, abs(d))
                lags.append((d / math.degrees(rate)) * 1000.0)
            diffs.append(abs(d))

            if el >= nxt:
                nxt = el + args.rate
                print('  t=%5.1fs  /odom=%+8.2f°  EKF=%+8.2f°  差=%+6.2f°'
                      % (el, math.degrees(wrap(o[2] - o0[2])),
                         math.degrees(wrap(e[2] - e0[2])), d))
    except KeyboardInterrupt:
        pass

    o, e = pose_of(st['odom']), pose_of(st['ekf'])
    print('\n=========== 总结（相对起点）===========')
    for name, a, b in (('/odom            ', o0, o),
                       ('/odometry/filtered', e0, e)):
        print('  %s  Δx=%+.3f  Δy=%+.3f  Δyaw=%+.2f°'
              % (name, b[0] - a[0], b[1] - a[1],
                 math.degrees(wrap(b[2] - a[2]))))
    print('  共记录 %.1fs' % (time.time() - t_start))

    if lags:
        print('  EKF 与轮速 yaw 差（运动中）max = %.2f°' % max_diff)
        print('  折算滞后 = 平均 %.0f ms，最大 %.0f ms'
              % (sum(lags) / len(lags), max(lags)))
    else:
        print('  全程没有明显转动 → 未统计滞后（静止测试属正常）')
    if diffs:
        print('  末段 yaw 差 = %.2f°（≈0 表示无偏置，只是滞后）' % max(diffs[-40:]))

    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
