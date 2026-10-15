"""节点层：GoalPrecheck 预检 + PhoneTeleop（原 phone_teleop.py 逐字平移）"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
from robot_ui.state_store import ModeMgr   # 控制权互斥（Task 5：四屏并发抢 /cmd_vel）
import argparse
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    import cv2
    import rclpy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.action import FollowWaypoints
    from sensor_msgs.msg import CompressedImage, Image   # 视觉 HUD：/image_det(画框) + /image_raw(原始)
    from cv_bridge import CvBridge                       # 无 yolo 时把 /image_raw 编成 JPEG
except ImportError as e:
    raise SystemExit(
        '导入模块失败：%s\n提示：先 source 环境（或用 ~/.bash_aliases 里的别名 phone）\n'
        '  source /opt/ros/humble/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

from map_viewer import MapViewer      # 复用地图订阅 + 渲染（同目录）
from save_map import DEFAULT_SAVE_DIR, save_pgm_yaml, serialize_posegraph
from robot_ui.patrol import PatrolRunner   # 屏3 巡逻后端（方案A：顺序 NavigateToPose）

CMD_TIMEOUT = 0.6                     # 看门狗：多久没收到指令就发零速度
STREAM_FPS = 8                        # 推给手机的帧率（地图流）
CAM_FPS = 10                          # 相机画框流推送帧率（帧源端 rknn_infer 也是 10fps 节流）
RANGE_MIN, RANGE_MAX = 1.0, 20.0      # 缩放范围（米）
PAN_MAX = 10.0                        # 视窗中心相对车的最大偏移（米）


import goal_nav as gn          # noqa: E402（与本文件同目录：预检直接复用 nav 的规划代码）


class GoalPrecheck:
    """点选目标前的"预检"：用与 `nav` **同一份规划代码**在本地试算一遍（不发任何指令）

    复用 `goal_nav` 的 `load_map` / `build_traversable` / `blocked_mask` / `clearance_cost` /
    `GoalNav.plan()` / `follow_px_path()` / `path_clearance_stats()`，起点取手机侧当前位姿，
    所以"预检通过"≈"nav 也能规划出同一条路径"。

    地图/代价图只加载一次；任何异常都降级为"不预检"，**绝不阻塞点选**。
    注意：预检用的是**默认参数**（inflation 0.15 / prefer-open 3 / min-obstacle-cells 0）；
    如果 `nav` 起了非默认参数，结论可能略有出入。
    """

    def __init__(self, map_prefix, inflation=0.15, prefer_open=3.0, open_dist=0.8,
                 min_obstacle_cells=0, smooth_iter=2):
        self.m = gn.load_map(map_prefix)
        self.trav = gn.build_traversable(self.m, inflation, False, min_obstacle_cells)
        self.blocked, _ = gn.blocked_mask(self.m, False, min_obstacle_cells)
        self.cost = gn.clearance_cost(self.m, self.blocked, open_dist, prefer_open)
        self.d_obs = gn.dist_to_obstacles(self.m, self.blocked)
        self.smooth_iter = smooth_iter
        self.inflation = inflation

    class _Shim:
        """GoalNav.plan() 只用到这几个属性 / get_logger()"""
        def __init__(self, m, trav, blocked, cost, pose):
            self.m, self.trav, self.blocked, self.cost, self.pose = m, trav, blocked, cost, pose

        def get_logger(self):
            class _L:
                def warn(self, msg):
                    pass

                def info(self, msg):
                    pass
            return _L()

    def check(self, start_xy, goal_xy):
        """返回 dict(ok, msg, dist, min_clear, narrow)；失败时 msg = 规划失败原因"""
        shim = self._Shim(self.m, self.trav, self.blocked, self.cost,
                          (float(start_xy[0]), float(start_xy[1]), 0.0))
        path, why = gn.GoalNav.plan(shim, (float(goal_xy[0]), float(goal_xy[1])))
        if path is None:
            return {'ok': False, 'msg': why}
        px, _ = gn.follow_px_path(self.m, self.trav, path, 0.1, self.smooth_iter)
        mn, avg, narrow = gn.path_clearance_stats(self.m, self.d_obs, px)
        return {'ok': True, 'msg': 'ok', 'dist': gn.path_length_m(self.m, px),
                'min_clear': mn, 'avg': avg, 'narrow': narrow}


class PhoneTeleop(MapViewer):
    """在地图查看器基础上：发 /cmd_vel、设/清导航目标（/goal_pose）、保存地图、网页服务状态"""

    def __init__(self, view_range, vmax, wmax, topic='/cmd_vel',
                 save_dir=DEFAULT_SAVE_DIR, goal_topic='/goal_pose',
                 precheck=True, map_prefix=None):
        super().__init__(view_range)
        self.topic = topic
        self.vmax = vmax
        self.wmax = wmax
        self.save_dir = save_dir      # 保存地图的目录（默认 ~/RobotCode/04_map）
        self.lock = threading.Lock()
        self.modes = ModeMgr()        # 控制权互斥（四屏并发用）
        self.patrol = PatrolRunner(self)   # 巡逻分发器（Task 6：/patrol/* 后端）
        self.cmd = (0.0, 0.0)         # 最近一次下发的 (vx, wz)
        self.last_cmd_t = 0.0         # 最近一次收到网页指令的时刻
        self.n_cmd = 0
        self.last_save = None         # 最近一次保存的文件名前缀
        self.odom_vel = None          # (vx, wz) 底盘回报的实际速度
        self.goal = None              # 最近一次设置的目标 (x, y, 时刻)，用于画面标注
        self.n_goal = 0
        # P34 多点导航（10 方案 A2）：航点队列 + waypoint_follower 的 FollowWaypoints
        self.wp_queue = []            # [(x, y), ...] 世界坐标，排队模式点地图追加
        self.wp_running = False       # FollowWaypoints 正在执行
        self.wp_cur = -1              # 当前航点序号（feedback 回报，0 起）
        self.wp_gh = None             # goal handle（取消用）
        self.wp_res_fut = None        # 结果 future（watch 线程轮询）
        self.wp_client = None         # ActionClient（惰性创建）
        self.wp_n = 0                 # 出发时的航点总数（进度显示用）
        self.view_range_cur = float(view_range)   # 当前视野（网页缩放会改它）
        self.view_off = (0.0, 0.0)                # 视窗中心相对车的偏移（米）
        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        # 点选预检：加载地图 + 代价图（只做一次）；失败只降级、不影响其它功能
        self.precheck = None
        if precheck:
            try:
                self.precheck = GoalPrecheck(os.path.expanduser(map_prefix or gn.MAP_DEFAULT))
                self.get_logger().info('预检就绪：%s（与 nav 同一份规划代码）' % (map_prefix or gn.MAP_DEFAULT))
            except Exception as e:
                self.get_logger().warn('预检不可用（点选将跳过预检）：%s' % e)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        # 视觉 HUD（《13》P3）：优先 rknn_infer 的画框 JPEG（/image_det，10fps）；
        # 没跑 yolo（robotnav --nav2 --ui）时自动降级到 /image_raw 原始画面（本地按 CAM_FPS 节流编码）
        self.det_jpeg = b''
        self.det_t = 0.0              # 最近一帧画框图的时刻（判断"有没有检测流"用）
        self.det_n = 0                # 收到的帧数（/state 展示链路状态用）
        self.raw_jpeg = b''
        self.raw_t = 0.0
        self.raw_n = 0
        self.det_lock = threading.Lock()
        self.bridge = CvBridge()
        self.create_subscription(
            CompressedImage, '/image_det', self.on_det_img, 1)
        self.create_subscription(
            Image, '/image_raw', self.on_raw_img, 1)

    def on_det_img(self, msg):
        with self.det_lock:
            self.det_jpeg = bytes(msg.data)
            self.det_t = time.time()
            self.det_n += 1

    def on_raw_img(self, msg):
        """原始相机帧 → JPEG（节流到 CAM_FPS，避免 30fps 全编码白烧 CPU）"""
        now = time.time()
        if now - self.raw_t < 1.0 / CAM_FPS:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        except Exception:
            return
        if not ok:
            return
        with self.det_lock:
            self.raw_jpeg = buf.tobytes()
            self.raw_t = now
            self.raw_n += 1

    def cam_src(self):
        """/cam.mjpg 当前画面来源：'det'（yolo 画框）/ 'raw'（原始画面）/ 'none'（没画面）"""
        now = time.time()
        with self.det_lock:
            if self.det_jpeg and now - self.det_t < 3.0:
                return 'det'
            if self.raw_jpeg and now - self.raw_t < 3.0:
                return 'raw'
        return 'none'

    def cam_jpeg(self):
        """取最近一帧画面 JPEG：有检测就用画框图，否则用原始相机画面（无则 b''，/cam.mjpg 空转等待）"""
        if self.cam_src() == 'raw':
            with self.det_lock:
                return self.raw_jpeg
        with self.det_lock:
            return self.det_jpeg

    def on_odom(self, msg):
        self.odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def set_cmd(self, vx, wz):
        """裁剪并发布速度；返回实际下发值"""
        vx = max(-self.vmax, min(self.vmax, vx))
        wz = max(-self.wmax, min(self.wmax, wz))
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        with self.lock:
            self.pub.publish(t)
            self.cmd = (vx, wz)
            self.last_cmd_t = time.time()
            self.n_cmd += 1
        return vx, wz

    # ---------- 地图视图（缩放/平移）----------
    def set_view(self, range_m=None, dx=None, dy=None, reset=False):
        """网页缩放/平移：range 1~20 m、偏移 ±10 m；reset=True 复位（回车上 + 初始视野）

        缩放是**服务端重渲染**（不是放大图片）：放大后依然清晰，点选坐标也自动跟着缩放走。
        """
        with self.lock:
            if reset:
                self.view_range_cur = float(self.view_range)
                self.view_off = (0.0, 0.0)
            if range_m is not None and range_m == range_m:
                self.view_range_cur = max(RANGE_MIN, min(RANGE_MAX, float(range_m)))
            ox, oy = self.view_off
            if dx is not None and dx == dx:
                ox = max(-PAN_MAX, min(PAN_MAX, float(dx)))
            if dy is not None and dy == dy:
                oy = max(-PAN_MAX, min(PAN_MAX, float(dy)))
            self.view_off = (ox, oy)
            cur = (self.view_range_cur, ox, oy)
        return {'ok': True, 'range': round(cur[0], 3),
                'dx': round(cur[1], 3), 'dy': round(cur[2], 3)}

    def view_offset(self):
        """当前平移偏移 (dx, dy)：render 会把它叠加到"车位置（无 TF 时地图中心）"上"""
        with self.lock:
            return self.view_off

    # ---------- 目标点（N4：手机网页点选导航目标）----------
    def _cell_of(self, x, y):
        """世界坐标 → 地图格值；'out'=出图，None=还没收到 /map"""
        m = self.map_msg
        if m is None:
            return None
        res = m.info.resolution
        w, h = m.info.width, m.info.height
        gx = int((x - m.info.origin.position.x) / res)
        gy = int((y - m.info.origin.position.y) / res)
        if not (0 <= gx < w and 0 <= gy < h):
            return 'out'
        return m.data[gy * w + gx]

    def set_goal(self, u=None, v=None, x=None, y=None):
        """设置导航目标并发 /goal_pose

        u,v  = 网页画面像素（由板子用渲染时的视窗参数反算世界坐标，缩放/平移后同样准确）
        x,y  = 也可直接给世界坐标（便于命令行/脚本自测）
        返回网页用的 dict：{'ok', 'msg', 'x', 'y', 'sub'}
        """
        if x is None or y is None:
            lv = self.last_view
            if lv is None:
                return {'ok': False, 'msg': '还没有地图画面（/map 或 TF 没到），稍等再点'}
            x0, y1, s, size = lv
            if u is None or v is None or not (0 <= u < size and 0 <= v < size):
                return {'ok': False, 'msg': '点到了画面外'}
            x, y = x0 + u / s, y1 - v / s
        self.set_cmd(0.0, 0.0)                      # 先停一帧，避免与摇杆指令打架
        # ── 预检：用与 nav 同一份规划代码本地试算（不发任何指令）──
        pre, pre_note = None, ''
        if self.precheck is not None:
            pose = self.robot_pose()
            if pose is None:
                pre_note = '｜预检跳过（还没有位姿）'
            else:
                try:
                    pre = self.precheck.check((pose[0], pose[1]), (x, y))
                except Exception as e:
                    pre = None
                    self.get_logger().warn('预检异常（跳过）：%s' % e)
        if pre is not None and not pre['ok']:
            m = '✗ 预检未通过：%s —— 请换个点（确要强行下发：启动 phone 时加 --no-precheck）' % pre['msg']
            self.get_logger().warn('目标 (%.2f, %.2f) 预检失败：%s' % (x, y, pre['msg']))
            return {'ok': False, 'msg': m, 'x': round(float(x), 2), 'y': round(float(y), 2),
                    'sub': 0}
        if pre is not None:
            pre_note = ('｜预检 ✓ 路径 %.2f m，离障碍最近 %.2f m（平均 %.2f）%s'
                        % (pre['dist'], pre['min_clear'], pre['avg'],
                           ' ⚠ 偏紧（<0.12 = 车半宽）' if pre['min_clear'] < 0.12 else ''))
        t = PoseStamped()
        t.header.frame_id = 'map'
        t.header.stamp = self.get_clock().now().to_msg()
        t.pose.position.x, t.pose.position.y = float(x), float(y)
        t.pose.orientation.w = 1.0                  # 朝向留给 goal_nav（可用 --goal x y yaw 再转）
        self.goal_pub.publish(t)
        self.goal = (float(x), float(y), time.time())
        with self.lock:
            self.n_goal += 1
        cell = self._cell_of(x, y)
        sub = self.goal_pub.get_subscription_count()
        if pre is not None:
            note = pre_note                     # 预检结论比"单格判据"更可信
        elif cell is None:
            note = '（还没收到 /map）'
        elif cell == 'out':
            note = '（在地图范围外，规划会失败）'
        elif cell != 0:
            note = '（注意：该点不是空闲区，规划可能失败）'
        else:
            note = ''
        if sub == 0:
            note += '｜⚠ /goal_pose 当前无订阅者：先另开终端跑 nav --listen'
        msg = '目标已设：x=%.2f y=%.2f%s' % (x, y, note)
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'x': round(float(x), 2), 'y': round(float(y), 2),
                'sub': sub}

    def goto_home(self):
        """一键回原点：目标 = 地图原点 (0, 0)，朝向 0°（= 建图开局时车的位置与车头方向）

        定位模式下"原点"就是**建图那次开局车的实际位置/朝向**（见 08 方案 N1），
        所以到点后车头也会转到 0°（地图 +x），与建图开局一致。
        若定位漂移导致原点对不上实际地面标记，先按 08 方案 N1 用 /initialpose 修正。
        """
        info = self.set_goal(x=0.0, y=0.0)
        if info.get('ok'):
            info['msg'] = '回原点 → ' + info['msg']
        return info

    def clear_goal(self):
        """清除目标，并让导航**就地停车**

        做法：把"车当前位姿"当成一个新目标发出去 —— 导航侧会立刻判定"已到达"、
        回到等待状态（它下一拍就不会再发速度指令了）。这比在手机上按「停 止」可靠：
        那个按钮只是把速度置零，导航进程下一轮照样会继续发指令。
        """
        pose = self.robot_pose()
        stopped = False
        if pose is not None and self.goal_pub.get_subscription_count() > 0:
            t = PoseStamped()
            t.header.frame_id = 'map'
            t.header.stamp = self.get_clock().now().to_msg()
            t.pose.position.x, t.pose.position.y = pose[0], pose[1]
            t.pose.orientation.w = 1.0
            self.goal_pub.publish(t)
            stopped = True
        self.set_cmd(0.0, 0.0)
        self.goal = None
        msg = '目标已清除' + ('（已让导航就地停车、回到等待状态）' if stopped else '')
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'stopped': stopped}

    # ---------------- 多点导航（10 方案 A2）：队列 + FollowWaypoints ----------------
    def _wp_client(self):
        if self.wp_client is None:
            from rclpy.action import ActionClient
            self.wp_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        return self.wp_client

    def wp_add(self, u=None, v=None, x=None, y=None):
        """多点模式：把一个点加入队列（不停车、不下发；出发时交给 waypoint_follower）"""
        if x is None or y is None:
            lv = self.last_view
            if lv is None:
                return {'ok': False, 'msg': '还没有地图画面，稍等再点'}
            x0, y1, s, size = lv
            if u is None or v is None or not (0 <= u < size and 0 <= v < size):
                return {'ok': False, 'msg': '点到了画面外'}
            x, y = x0 + u / s, y1 - v / s
        x, y = float(x), float(y)
        # 逐段预检：起点 = 队列里上一个点（首点用车的当前位姿）——与单目标同一份规划代码
        if self.wp_queue:
            start = self.wp_queue[-1]
        else:
            pose = self.robot_pose()
            start = (pose[0], pose[1]) if pose is not None else None
        note = ''
        if self.precheck is not None and start is not None:
            try:
                pre = self.precheck.check(start, (x, y))
                if pre is not None and not pre['ok']:
                    m = '✗ 第 %d 点预检未通过：%s —— 请换个点' % (len(self.wp_queue) + 1, pre['msg'])
                    self.get_logger().warn(m)
                    return {'ok': False, 'msg': m, 'n': len(self.wp_queue)}
                if pre is not None:
                    note = '｜路径 %.2f m 离障 %.2f' % (pre['dist'], pre['min_clear'])
            except Exception as e:
                self.get_logger().warn('多点预检异常（跳过）：%s' % e)
        self.wp_queue.append((x, y))
        msg = '第 %d 点已排：x=%.2f y=%.2f%s' % (len(self.wp_queue), x, y, note)
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'n': len(self.wp_queue),
                'x': round(x, 2), 'y': round(y, 2)}

    def wp_go(self):
        """出发：把整个队列交给 waypoint_follower（内部逐段调 navigate_to_pose）"""
        if self.wp_running:
            return {'ok': False, 'msg': '多点导航正在执行（先「清除目标」取消再重排）'}
        if not self.wp_queue:
            return {'ok': False, 'msg': '队列为空：先在多点模式下点地图排队'}
        ac = self._wp_client()
        if not ac.wait_for_server(timeout_sec=3.0):
            return {'ok': False,
                    'msg': 'follow_waypoints 服务不在（robotnav --nav2 未起 nav2 栈？）'}
        g = FollowWaypoints.Goal()
        for (x, y) in self.wp_queue:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x, ps.pose.position.y = x, y
            ps.pose.orientation.w = 1.0
            g.poses.append(ps)
        self.set_cmd(0.0, 0.0)
        self.goal = None
        fut = ac.send_goal_async(g, feedback_callback=self._wp_feedback)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 8.0:
            time.sleep(0.05)
        if not fut.done():
            return {'ok': False, 'msg': 'follow_waypoints 响应超时'}
        gh = fut.result()
        if not gh.accepted:
            return {'ok': False, 'msg': '航点任务被 nav2 拒绝'}
        self.wp_gh = gh
        self.wp_running = True
        self.wp_cur = -1
        self.wp_res_fut = gh.get_result_async()
        n = len(self.wp_queue)
        self.wp_n = n
        self.wp_t0 = time.time()
        self.get_logger().info('多点导航出发：%d 个航点' % n)
        threading.Thread(target=self._wp_watch, args=(gh, list(self.wp_queue)),
                         daemon=True).start()
        return {'ok': True, 'msg': '多点出发：%d 个航点（进度见地图与终端）' % n, 'n': n}

    def _wp_feedback(self, fb):
        # nav2_msgs/action/FollowWaypoints 的反馈字段是 current_waypoint（Humble）；
        # 旧称 current_pose_idx 已不存在——直接取属性会让 executor 抛异常、spin 线程整个死掉
        # （症状：页面上的车不再跟着走）。用 getattr 兼容 + 兜底，任何解析失败都不致命。
        try:
            idx = int(getattr(fb.feedback, 'current_waypoint',
                              getattr(fb.feedback, 'current_pose_idx', 0)))
        except Exception as e:
            self.get_logger().warn('航点进度反馈解析失败（已忽略）：%s' % e)
            return
        if idx != self.wp_cur:
            self.wp_cur = idx
            self.get_logger().info('多点导航：正在前往第 %d/%d 点' % (idx + 1, len(self.wp_queue)))

    def _wp_watch(self, gh, pts):
        """后台线程：轮询结果 future（读布尔不碰 executor），打印每点误差与总耗时"""
        t0 = time.time()
        while rclpy.ok() and not self.wp_res_fut.done() and time.time() - t0 < 1800:
            time.sleep(0.5)
        self.wp_running = False
        self.modes.release('web-nav')       # 多点导航结束（完成/超时）→ 释放 NAV，勿挡巡逻
        if not self.wp_res_fut.done():
            self.get_logger().warn('多点导航 30 分钟超时，取消')
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            res = self.wp_res_fut.result().result
            missed = list(res.missed_waypoints)
        except Exception as e:
            self.get_logger().error('多点导航结果读取失败：%s' % e)
            return
        if missed:
            self.get_logger().warn('多点导航完成：未到达的航点序号 %s' % missed)
        else:
            self.get_logger().info('✓ 多点导航全部 %d 点到达，总耗时 %.0f s'
                                   % (len(pts), time.time() - self.wp_t0))

    def wp_clear(self):
        """清空队列；若多点导航在执行则取消（车会在当前航点附近停下）"""
        q = len(self.wp_queue)
        self.wp_queue = []
        cancelled = False
        if self.wp_running and self.wp_gh is not None:
            try:
                self.wp_gh.cancel_goal_async()
                cancelled = True
            except Exception:
                pass
        self.wp_running = False
        self.set_cmd(0.0, 0.0)
        msg = '队列已清空（%d 点）' % q + ('｜多点导航已取消' if cancelled else '')
        self.get_logger().info(msg)
        return {'ok': True, 'msg': msg, 'cancelled': cancelled}

    def _world_px(self, x, y, size):
        """世界坐标 → 当前画面的像素位置（不在画面内返回 None）"""
        if self.last_view is None:
            return None
        x0, y1, s, sz = self.last_view
        px = int(round((x - x0) * s))
        py = int(round((y1 - y) * s))
        if not (0 <= px < sz and 0 <= py < sz):
            return None
        return px, py

    def goal_px(self, size):
        """目标点 → 画面像素（供推流标注）；不在画面内返回 None"""
        if self.goal is None:
            return None
        return self._world_px(self.goal[0], self.goal[1], size)

    def car_px(self, size):
        """车 → 画面像素（缩放/平移后车不在画面中心，标注连线要从这里出发）"""
        pose = self.robot_pose()
        if pose is None:
            return None
        return self._world_px(pose[0], pose[1], size)

    def save_map(self):
        """保存当前 SLAM 图（PGM/YAML + 建图序列），文件名带时间戳，存到 self.save_dir

        返回给网页的 dict：{'ok': bool, 'msg': str, 'name': str, 'dir': str}
        """
        m = self.map_msg
        if m is None:
            return {'ok': False, 'msg': '还没收到 /map —— SLM 没在运行？'}

        try:
            os.makedirs(self.save_dir, exist_ok=True)
        except OSError as e:
            return {'ok': False, 'msg': '无法创建目录 %s（%s）' % (self.save_dir, e)}

        name = time.strftime('map_%Y%m%d_%H%M%S')
        prefix = os.path.join(self.save_dir, name)
        written, notes = [], []

        try:
            written += save_pgm_yaml(m, prefix)
            notes.append('pgm+yaml(%dx%d)' % (m.info.width, m.info.height))
        except Exception as e:
            notes.append('pgm 失败：%s' % e)

        ok_pg, info = serialize_posegraph(self, prefix, timeout=30.0, spin=False)
        if ok_pg:
            written += [prefix + '.posegraph', prefix + '.data']
            notes.append('posegraph')
        else:
            notes.append('序列失败：%s' % info)

        sizes = ' ｜ '.join('%s %.0fKB' % (os.path.basename(p), os.path.getsize(p) / 1024.0)
                            for p in written if os.path.exists(p))
        self.get_logger().info('保存地图 → %s（%s）' % (prefix, ' + '.join(notes)))
        self.last_save = name
        return {'ok': bool(written), 'msg': '%s ｜ %s' % (name, sizes or '未写出文件'),
                'name': name, 'dir': self.save_dir}

    def watchdog(self):
        """超时未收到网页指令 → 主动发零速度（第 2 层保险）"""
        while rclpy.ok():
            time.sleep(0.1)
            with self.lock:
                stale = self.last_cmd_t > 0 and time.time() - self.last_cmd_t > CMD_TIMEOUT
                moving = abs(self.cmd[0]) > 1e-6 or abs(self.cmd[1]) > 1e-6
            if stale and moving:
                self.set_cmd(0.0, 0.0)
                self.get_logger().warn('网页指令超时 %.1fs，已自动停车' % CMD_TIMEOUT)

    def state(self):
        pose = self.robot_pose()
        with self.lock:
            cmd_age = time.time() - self.last_cmd_t if self.last_cmd_t else 1e9
            vx, wz = self.cmd
            n = self.n_cmd
            rng, ox, oy = self.view_range_cur, self.view_off[0], self.view_off[1]
        m = self.map_msg
        return {
            'pose': ('x=%.2f y=%.2f yaw=%.0f°' % (pose[0], pose[1], math.degrees(pose[2])))
                    if pose else None,
            'vel': ('cmd=%.2f m/s %.2f rad/s ｜ 实际=%.2f m/s %.2f rad/s'
                    % (vx, wz, self.odom_vel[0], self.odom_vel[1])) if self.odom_vel
                   else ('cmd=%.2f m/s %.2f rad/s' % (vx, wz)),
            'cmd_age': round(cmd_age, 2),
            'n_cmd': n,
            'map': ('%dx%d @%.3fm' % (m.info.width, m.info.height, m.info.resolution))
                   if m else None,
            'goal': ('x=%.2f y=%.2f%s' % (self.goal[0], self.goal[1],
                                          '' if self.goal_pub.get_subscription_count()
                                          else ' ⚠无监听')) if self.goal else None,
            # P34 多点导航状态（HUD「目标」行显示队列/进度）
            'wp': ('排队 %d 点' % len(self.wp_queue)) if self.wp_queue
                  else ('执行中 %d/%d' % (self.wp_cur + 1, self.wp_n)
                        if self.wp_running else None),
            'mode': self.modes.snapshot(),      # {'mode','holder'}：四屏互斥状态
            'patrol': self.patrol.status(),     # 屏3 数据源：cur/total/per/elapsed/eta/running/paused
            'view': ('%.1f m%s' % (rng, ('，偏移 %.1f/%.1f' % (ox, oy))
                                   if abs(ox) > 0.05 or abs(oy) > 0.05 else '')),
            'view_range': round(rng, 3),        # 给网页初始化用（数值）
            'last_save': self.last_save,
            # 相机画面来源：det=yolo 画框 / raw=原始画面（没跑 yolo）/ none=无画面（cam_pub 没起）
            'cam': {'src': self.cam_src(), 'det_n': self.det_n, 'raw_n': self.raw_n},
        }
