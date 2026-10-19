"""巡逻分发器（方案A）：顺序逐点发 NavigateToPose + 逐点停留

对应网页屏3（patrol.html）的数据源。与旧 /wp/*（FollowWaypoints 连发）并存：
本模块支持逐点停留（dwell）、逐点状态、失败跳过与圈数循环。

与 node_teleop.wp_go 同一套异步模式：send_goal_async + 轮询 future，
节点由 app.py 的 rclpy.spin 线程驱动——本模块**绝不自建 executor/spin**。
"""
import time
import threading
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

GOAL_ACCEPT_TIMEOUT = 8.0     # 等待 nav2 接受目标（秒）
POINT_TIMEOUT = 600.0         # 单点导航超时（秒）
SPEED_EST = 0.15              # ETA 估算用的平均速度（m/s，与实机速度上限同量级）
GOAL_SUCCEEDED = 4            # action_msgs/GoalStatus：SUCCEEDED
TF_MAX_AGE = 2.0               # 定位 TF 超过 2 秒不得出发或确认到站
ARRIVAL_TOLERANCE = 0.30       # Nav2 容差 0.20 m，另留 0.10 m 观测余量
TOTAL_CAP = 1800.0            # 巡逻总时长上限（秒），防无人值守无限圈


class PatrolRunner:
    """顺序巡逻：逐点 NavigateToPose → 到点停留 dwell 秒 → 下一点（可循环）

    状态机：待命 → running（_run 后台线程）→ 收尾（停车 + release('patrol')）。
    所有对 self.st / 标志位的写都在 self.lock 内；Event 本身原子。
    """

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.pts = []                       # [{'x','y','name','dwell'}]
        self._nav_s = []                    # 与 pts 对齐：每点导航耗时估算（秒）
        self.loop = False
        self.running = False
        self.paused = False
        self.cancel_evt = threading.Event()
        self.skip_evt = threading.Event()
        self.gh = None                      # 当前目标 handle（取消用）
        self.client = None                  # ActionClient（惰性创建）
        self.st = {'cur': 0, 'total': 0, 'per': [], 'elapsed': 0.0,
                   'eta': 0.0, 'msg': '待命'}

    # ---------- 内部工具 ----------
    def _client(self):
        """惰性创建 ActionClient（话题名与 nav2 栈的 NavigateToPose 一致）"""
        if self.client is None:
            from rclpy.action import ActionClient
            self.client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        return self.client

    def _eta_estimate(self, from_idx=0):
        """剩余 ETA 粗估：Σ(每点 导航估算 + 停留)；导航估算 = 预检路径长/SPEED_EST，缺省 30s"""
        total = 0.0
        for j in range(from_idx, len(self.pts)):
            total += (self._nav_s[j] if j < len(self._nav_s) else 30.0) \
                     + self.pts[j]['dwell']
        return total

    # ---------- 装载 / 查询 ----------
    def load(self, pts, loop):
        """装载巡逻点并逐段预检（任一点失败 → 整批拒绝）；运行中拒绝

        pts 元素：{'x','y','dwell','name'}（dwell/name 可缺省）。
        预检与 wp_add 同一份逻辑：起点 = 上一点，首点 = 车当前位姿；
        precheck 为 None 或无位姿时跳过预检、不阻塞。
        """
        with self.lock:
            if self.running:
                return {'ok': False, 'msg': '巡逻进行中，先「取消」再重新装载'}
        start = None
        if self.node.precheck is not None:
            pose = self.node.robot_pose()
            start = (pose[0], pose[1]) if pose is not None else None
        clean, nav_s = [], []
        for i, p in enumerate(pts):
            try:
                x, y = float(p['x']), float(p['y'])
                d = p.get('dwell', 3.0)
                dwell = 3.0 if d is None else float(d)
            except (TypeError, ValueError):
                return {'ok': False, 'msg': '第 %d 点 x/y/dwell 不是数字' % (i + 1)}
            if not (x == x and y == y):         # NaN 一律拒绝
                return {'ok': False, 'msg': '第 %d 点坐标无效' % (i + 1)}
            dwell = max(0.0, min(60.0, dwell))  # 夹到 0~60s
            name = str(p.get('name') or ('点%d' % (i + 1)))
            seg = 30.0                          # 预检不可用时的单点兜底估算
            if self.node.precheck is not None and start is not None:
                try:
                    pre = self.node.precheck.check(start, (x, y))
                except Exception:
                    pre = None                  # 预检异常：降级不阻塞（与 wp_add 同策略）
                if pre is not None and not pre['ok']:
                    return {'ok': False,
                            'msg': '✗ 第 %d 点预检未通过：%s —— 请调整巡逻点' % (i + 1, pre['msg'])}
                if pre is not None and pre.get('ok'):
                    seg = float(pre.get('dist', 0.0)) / SPEED_EST
            nav_s.append(seg)
            clean.append({'x': x, 'y': y, 'dwell': dwell, 'name': name})
            start = (x, y)                      # 下一段起点 = 本点
        if not clean:
            return {'ok': False, 'msg': '巡逻点为空'}
        with self.lock:
            if self.running:                    # 复查：预检耗时窗口里 go() 可能已启动（TOCTOU）
                return {'ok': False, 'msg': '巡逻执行中，禁止装载'}
            self.pts = clean
            self._nav_s = nav_s
            self.loop = bool(loop)
        return {'ok': True, 'msg': '已装载 %d 点' % len(clean), 'n': len(clean)}

    def points(self):
        """当前装载的巡逻点（给网页回显）"""
        with self.lock:
            return {'ok': True, 'pts': [dict(p) for p in self.pts], 'loop': self.loop}

    def status(self):
        """屏3 数据源：cur/total/per/elapsed/eta + running/paused/loop"""
        with self.lock:
            st = dict(self.st)
            st['per'] = list(self.st['per'])
            st['running'] = self.running
            st['paused'] = self.paused
            st['loop'] = self.loop
            return st

    # ---------- 控制入口（HTTP 线程调用）----------
    def go(self):
        """出发：claim PATROL → 起后台线程 _run

        force=True：用户明确点「开始巡逻」即接管 NAV/TELEOP（残留的 NAV 不该挡巡逻）；
        force_unless='PATROL'：仅 PATROL 本身占用时拒绝（防并发 go）。
        """
        with self.lock:
            if self.running:
                return {'ok': False, 'msg': '巡逻已在执行中'}
        if not self.pts:
            return {'ok': False, 'msg': '巡逻点为空：先 /patrol/load 装载'}
        if self.node.robot_pose(max_age=TF_MAX_AGE) is None:
            return {'ok': False, 'msg': '定位 TF 未更新（超过 2 秒），请检查 slam_toolbox 后再出发'}
        ok, mmsg = self.node.modes.claim('patrol', 'PATROL',
                                         force=True, force_unless='PATROL')
        if not ok:
            return {'ok': False, 'msg': '✗ ' + mmsg}
        if not self._client().wait_for_server(3.0):
            self.node.modes.release('patrol')
            return {'ok': False, 'msg': 'navigate_to_pose 服务不在（nav2 栈没起？）'}
        with self.lock:
            if self.running:                    # 双检：两个 go 并发时只放一个
                self.node.modes.release('patrol')
                return {'ok': False, 'msg': '巡逻已在执行中'}
            self.cancel_evt.clear()
            self.skip_evt.clear()
            self.paused = False
            self.running = True
        try:
            threading.Thread(target=self._run, daemon=True).start()
        except RuntimeError as e:
            # 线程耗尽等启动失败：PATROL 已 claim 但无线程走 finally，必须在此收尾
            self.node.set_cmd(0.0, 0.0)
            self.node.modes.release('patrol')
            with self.lock:
                self.running = False
                self.paused = False
                self.gh = None
            return {'ok': False, 'msg': '线程启动失败：%s' % e}
        return {'ok': True, 'msg': '巡逻出发：%d 点（loop=%s）'
                % (len(self.pts), '开' if self.loop else '关')}

    def pause(self):
        """暂停：在「下一点出发前」生效（已发出的导航目标不受影响）"""
        with self.lock:
            if not self.running:
                return {'ok': False, 'msg': '未在巡逻中'}
            self.paused = True
        return {'ok': True, 'msg': '已暂停（当前点到站后不再出发，停留不计时暂停）'}

    def resume(self):
        with self.lock:
            if not self.running:
                return {'ok': False, 'msg': '未在巡逻中'}
            self.paused = False
        return {'ok': True, 'msg': '已恢复巡逻'}

    def skip(self):
        """跳过当前点剩余导航/停留（运行中才有效）"""
        with self.lock:
            if not self.running:
                return {'ok': False, 'msg': '未在巡逻中'}
        self.skip_evt.set()
        return {'ok': True, 'msg': '将跳过当前点（取消导航 / 提前结束停留）'}

    def cancel(self):
        """取消整趟巡逻；实际收尾（停车+release+running=False）统一在 _run 里做"""
        with self.lock:
            self.cancel_evt.set()
            gh = self.gh
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass
        return {'ok': True, 'msg': '取消中（收尾后自动释放 PATROL）'}

    # ---------- 后台执行 ----------
    def _send_goal(self, x, y):
        """向 navigate_to_pose 发单点目标；未被接受/响应超时 → None

        ⚠ goal 必须是 nav2_msgs/action/NavigateToPose 实例（内含 .pose），
        不能直接传 PoseStamped——rclpy 类型校验会抛 TypeError（空消息，难排查）。
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        fut = self._client().send_goal_async(goal)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < GOAL_ACCEPT_TIMEOUT:
            time.sleep(0.05)
        if not fut.done():
            return None
        gh = fut.result()
        if not gh.accepted:
            return None
        return gh

    def _wait_goal(self, gh, t0, goal_xy):
        """轮询结果 future（0.2s 一拍，POINT_TIMEOUT 总闸）

        每拍检查 cancel_evt → 'canceled'、skip_evt → cancel + 'skipped'；
        正常完成按 result.status 判定：SUCCEEDED → 'done'，其余 → 'failed'。
        """
        res_fut = gh.get_result_async()
        deadline = time.time() + POINT_TIMEOUT
        while not res_fut.done():
            if self.node.robot_pose(max_age=TF_MAX_AGE) is None:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return 'tf_stale'
            if self.cancel_evt.is_set():
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return 'canceled'
            if self.skip_evt.is_set():
                self.skip_evt.clear()           # 立即消费事件：防亚秒窗口内重复 skip 被静默吞掉
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return 'skipped'
            if time.time() > deadline:
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass
                return 'failed'
            with self.lock:
                self.st['elapsed'] = time.time() - t0
            time.sleep(0.2)
        if self.cancel_evt.is_set():            # 完成瞬间才置的取消：以取消为准
            return 'canceled'
        if self.skip_evt.is_set():              # 完成瞬间才置的 skip：消费即清
            self.skip_evt.clear()
            return 'skipped'
        try:
            status = res_fut.result().status
        except Exception:
            status = None
        if status is None or status != GOAL_SUCCEEDED:
            return 'failed'                     # fail-safe：异常/失败一律不算成功
        pose = self.node.robot_pose(max_age=TF_MAX_AGE)
        if pose is None:
            return 'tf_stale'
        error = math.hypot(pose[0] - goal_xy[0], pose[1] - goal_xy[1])
        if error > ARRIVAL_TOLERANCE:
            self.node.get_logger().error('Nav2 报到达但距目标 %.2fm，拒绝误判' % error)
            return 'arrival_mismatch'
        return 'done'

    def _run(self):
        """巡逻主循环（独立 daemon 线程）：逐点 发目标→等结果→停留；收尾统一在这里"""
        node = self.node
        t0 = time.time()
        nfail = 0
        end_kind = 'done'                       # done / canceled / capped / error
        try:
            node.set_cmd(0.0, 0.0)
            with self.lock:
                if self.cancel_evt.is_set():
                    end_kind = 'canceled'
                    return
                node.publish_control('nav')
                n = len(self.pts)
                self.st.update({'cur': 0, 'total': n, 'elapsed': 0.0,
                                'eta': self._eta_estimate(0), 'msg': '巡逻中'})
                self.st['per'] = ['pending'] * n
            lap = 0
            while not self.cancel_evt.is_set():
                if time.time() - t0 > TOTAL_CAP:    # 总时长上限：无人值守防无限圈
                    end_kind = 'capped'
                    with self.lock:
                        self.st['msg'] = '总时长超 30 分钟上限，自动终止'
                    break                           # 自然走 finally 收尾
                if lap > 0:                     # 循环模式：第 2 圈起逐点状态重置
                    with self.lock:
                        self.st['per'] = ['pending'] * n
                        self.st['eta'] = self._eta_estimate(0)
                i = 0
                while i < n:
                    if self.cancel_evt.is_set():
                        end_kind = 'canceled'
                        break
                    # 暂停门：暂停时原地等待（不撤目标、不丢进度）
                    while self.paused and not self.cancel_evt.is_set():
                        time.sleep(0.2)
                        with self.lock:
                            self.st['elapsed'] = time.time() - t0
                    if self.cancel_evt.is_set():
                        end_kind = 'canceled'
                        break
                    if self.skip_evt.is_set():
                        self.skip_evt.clear()   # 丢弃上一拍残留的 skip
                    pt = self.pts[i]
                    pt_t0 = time.time()
                    with self.lock:
                        self.st['cur'] = i
                        self.st['per'][i] = 'active'
                        self.st['eta'] = self._eta_estimate(i)   # 剩余点估算+剩余停留
                    if node.robot_pose(max_age=TF_MAX_AGE) is None:
                        with self.lock:
                            self.st['per'][i] = 'failed'
                        end_kind = 'localization_lost'
                        node.get_logger().error('定位 TF 过期，停止巡逻（第 %d 点）' % (i + 1))
                        break
                    gh = self._send_goal(pt['x'], pt['y'])
                    if gh is None:
                        with self.lock:
                            self.st['per'][i] = 'failed'
                        nfail += 1
                        node.get_logger().warn('巡逻第 %d 点：目标未被接受/响应超时，跳过' % (i + 1))
                        i += 1
                        continue
                    with self.lock:
                        self.gh = gh
                    res = self._wait_goal(gh, t0, (pt['x'], pt['y']))
                    with self.lock:
                        self.gh = None
                    if res == 'done':
                        with self.lock:
                            self.st['per'][i] = 'done'
                        node.get_logger().info('巡逻第 %d/%d 点到达，停留 %.0fs'
                                               % (i + 1, n, pt['dwell']))
                        # 逐点停留：0.2s 一拍，可被 skip/cancel 打断
                        dend = time.time() + pt['dwell']
                        while time.time() < dend:
                            if self.cancel_evt.is_set():
                                end_kind = 'canceled'
                                break
                            if self.skip_evt.is_set():
                                self.skip_evt.clear()   # 停留提前结束，不带去下一点
                                break
                            with self.lock:
                                self.st['elapsed'] = time.time() - t0
                                self.st['eta'] = max(0.0, dend - time.time()) \
                                                 + self._eta_estimate(i + 1)
                            time.sleep(0.2)
                    elif res == 'skipped':
                        with self.lock:
                            self.st['per'][i] = 'skipped'
                    elif res == 'canceled':
                        end_kind = 'canceled'
                        break
                    elif res in ('tf_stale', 'arrival_mismatch'):
                        with self.lock:
                            self.st['per'][i] = 'failed'
                        end_kind = 'localization_lost'
                        node.get_logger().error('巡逻第 %d 点定位异常/到站误判，停止后续航点' % (i + 1))
                        break
                    else:                       # 'failed'：失败跳过，继续下一点
                        with self.lock:
                            self.st['per'][i] = 'failed'
                        nfail += 1
                        node.get_logger().warn('巡逻第 %d 点导航失败/超时，跳过' % (i + 1))
                    if end_kind == 'canceled':
                        break
                    i += 1
                if end_kind in ('canceled', 'localization_lost') or self.cancel_evt.is_set():
                    if end_kind != 'localization_lost':
                        end_kind = 'canceled'
                    break
                if not self.loop:
                    break
                lap += 1
        except Exception as e:
            end_kind = 'error'
            try:
                node.get_logger().error('巡逻线程异常：%s' % e)
                import traceback
                traceback.print_exc()       # 完整堆栈进 nohup 日志，定位用
            except Exception:
                pass
        finally:
            # 收尾（finally 语义）：停车 + 释放 PATROL + 结束态
            node.set_cmd(0.0, 0.0)
            if self.cancel_evt.is_set():
                node.publish_control('stop')
            node.modes.release('patrol')
            with self.lock:
                self.running = False
                self.paused = False
                self.gh = None
                self.st['elapsed'] = time.time() - t0
                self.st['eta'] = 0.0
                if end_kind == 'done':
                    # 走完全程：进度推满（否则 cur 停在最后一点的 0 基索引 n-1，
                    # 页面会显示 "n-1 / n" 而列表已全部"已完成"——P35 实测）
                    self.st['cur'] = len(self.pts)
                elif any(s == 'active' for s in self.st['per']):
                    # 非正常走完（取消/超上限/异常）：正在走的点不会再有结果，标"已跳过"，
                    # 避免 pill 已是"待命"而列表还挂着"进行中"
                    self.st['per'] = ['skipped' if s == 'active' else s
                                      for s in self.st['per']]
                if end_kind == 'canceled':
                    self.st['msg'] = '已取消'
                elif end_kind == 'capped':
                    self.st['msg'] = '总时长超 30 分钟上限，自动终止'
                elif self.skip_evt.is_set():
                    self.st['msg'] = '已跳过终止'
                elif end_kind == 'error':
                    self.st['msg'] = '异常结束（失败 %d 点）' % nfail
                elif end_kind == 'localization_lost':
                    self.st['msg'] = '定位异常：第 %d 点未到达，已停止' % (self.st['cur'] + 1)
                elif nfail:
                    self.st['msg'] = '已完成（失败 %d 点）' % nfail
                else:
                    self.st['msg'] = '已完成'
