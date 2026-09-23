#!/usr/bin/env python3
"""odom_rec —— 实时记录 map 位姿 + yaw + /cmd_vel 到 /tmp/rec_live.log（每秒一条）。
robotnav 启动时自动拉起；errexp/诊断都读这个文件。
用法： python3 ~/RobotCode/01_Project/03_SLAM/odom_rec.py"""
import math
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from tf2_ros.buffer import Buffer

out = open('/tmp/rec_live.log', 'w', buffering=1)
rclpy.init()
n = Node('odom_rec')
buf = Buffer()
tf2_ros.TransformListener(buf, n)
v = [None]


def cb(m):
    v[0] = (round(m.linear.x, 3), round(m.angular.z, 3))


n.create_subscription(Twist, '/cmd_vel', cb, 10)
t0 = time.time()
last = -1
while True:
    rclpy.spin_once(n, timeout_sec=0.1)
    el = int(time.time() - t0)
    if el != last:
        last = el
        try:
            tf = buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            p = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))
            out.write('%4ds (%6.2f,%6.2f) yaw=%6.1f v=%s\n'
                      % (el, p.x, p.y, yaw, v[0]))
        except Exception as e:
            out.write('%4ds TF_FAIL %s\n' % (el, type(e).__name__))
