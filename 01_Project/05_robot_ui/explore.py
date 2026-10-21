"""ExploreRunner —— 前沿探索建图（P2-B1）

思路（17 号文档 P2：探索 = 导航 + 建图闭环）：
    在 /map 上找「前沿」= 空闲格子与未知格子相邻的地方
        ↓ 聚类，选最近/最大的一个
    当作导航目标发出去（走 nav2 的 NavigateToPose）
        ↓ 到达后重新检测
    直到没有前沿 → 保存地图

三条硬约束（与用户确认过的方案一致）：
  1. **不发 /cmd_vel**，只发目标 → 复用统一任务入口与安全链（P1）
  2. **续建 + 存档不切**：默认载入已有 posegraph 在原图上扩图；
     结果另存新版本前缀，**永不覆盖 robot_map.*、不自动激活**
  3. 停止必须走 `/stop` 语义（cancel → 停车 + 释放控制权）

⚠️ 续建前置校验：车的起始位姿必须在已有图覆盖内，否则 slam_toolbox 匹配失败
   会生成**连通孤岛**（旧图断开、定位崩溃）。
⚠️ 续建只在 **mapping 模式**可用（localization 已加载 map，deserialize_map 服务不在）。
"""
import math
import os
import threading
import time

import numpy as np

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from robot_ui.frontier_policy import observation_goals

MAP_DIR = os.path.expanduser('~/RobotCode/04_map')
MAP_BASE = os.path.join(MAP_DIR, 'robot_map')
TOTAL_CAP = 3600.0            # 一次探索总时长上限（秒），防跑飞
GOAL_TIMEOUT = 180.0          # 单个前沿目标的超时（秒）
MAX_FAIL = 5                  # 连续失败多少次就停
CLUSTER_MIN = 4               # 前沿连通域最小格数。4 邻接连通下，一扇 0.4 m 的门
                              # （5 cm/格 → 8 格宽）边界每边才 8 格，阈值 8 会漏掉窄门；
                              # 4 能滤掉 2×2 以下的雷达噪点，又不误杀窄门。
EXPLORE_TIMEOUT = None        # ModeMgr 超时：探索长期占控，不自动过期（同 PATROL）


def detect_frontiers(grid, min_cells=CLUSTER_MIN):
    """在栅格上找前沿并聚类。

    grid: int16 (h, w)，-1=未知 0=空闲 1..100=占据
    返回 [(cells, centroid_cell)] 列表，cells 是 [(col, row), ...]
    """
    free = (grid == 0)
    unknown = (grid == -1)
    if not free.any() or not unknown.any():
        return []

    # 前沿格 = 空闲 且 4 邻域里有未知
    adj = np.zeros(free.shape, dtype=bool)
    adj[:-1, :] |= unknown[1:, :]
    adj[1:, :] |= unknown[:-1, :]
    adj[:, :-1] |= unknown[:, 1:]
    adj[:, 1:] |= unknown[:, :-1]
    frontier = free & adj
    if not frontier.any():
        return []

    # 连通域（4 邻接，BFS）
    h, w = frontier.shape
    seen = np.zeros(frontier.shape, dtype=bool)
    clusters = []
    idx = np.argwhere(frontier)                 # [(row, col), ...]
    for r0, c0 in idx:
        if seen[r0, c0]:
            continue
        stack = [(r0, c0)]
        seen[r0, c0] = True
        comp = []
        while stack:
            r, c = stack.pop()
            comp.append((c, r))                 # 注意存 (col, row)
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and frontier[nr, nc] and not seen[nr, nc]:
                    seen[nr, nc] = True
                    stack.append((nr, nc))
        if len(comp) >= min_cells:
            xs = [c for c, _ in comp]
            ys = [r for _, r in comp]
            cx, cy = sum(xs) / len(xs), sum(ys) / len(ys)
            # 目标取离质心最近的**真实前沿格**（质心可能落在未知/障碍里）
            best = min(comp, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
            clusters.append((comp, best))
    return clusters


class ExploreRunner:
    """前沿探索执行器（仿 PatrolRunner：绝不自建 executor，finally 统一收尾）"""

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.running = False
        self.cancel_evt = threading.Event()
        self._client = None
        self._goal_handle = None
        self._continued = False
        self._starting = False
        self.st = {'running': False, 'mode': 'continue', 'frontiers': 0,
                   'visited': 0, 'goal': None, 'failed': 0,
                   'saved_as': None, 'msg': '未开始',
                   # 前沿点（世界坐标，最多 60 个），供探索屏在地图上叠加
                   'frontier_pts': []}

    # ---------------- 给 SSE 用 ----------------
    def status(self):
        with self.lock:
            return dict(self.st)

    def _set(self, **kw):
        with self.lock:
            self.st.update(kw)

    # ---------------- 控制入口 ----------------
    def go(self, mode='continue'):
        with self.lock:
            if self.running or self._starting:
                return {'ok': False, 'msg': '探索已在执行中'}
            self._starting = True
            self.cancel_evt.clear()
        try:
            return self._go(mode)
        except Exception as exc:
            self.node.modes.release('explore')
            return {'ok': False, 'msg': '探索启动失败：%s' % exc}
        finally:
            with self.lock:
                self._starting = False

    def _go(self, mode):
        """启动探索。续建载入旧图；fresh 使用本轮建图，不隐式删除地图。"""
        if mode not in ('continue', 'fresh'):
            return {'ok': False, 'msg': 'mode 必须是 continue/fresh'}
        ok, msg = self._check_mapping_mode()
        if not ok:
            return {'ok': False, 'msg': msg}
        if mode == 'fresh' and self._continued:
            return {'ok': False, 'msg': '本轮已载入旧图；从零建图请重启 robot --nav2 --ekf'}
        if getattr(self.node, 'raw_map_msg', None) is None:
            return {'ok': False, 'msg': '尚未收到原始 SLAM /map，不能使用编辑图开始探索'}
        if self.cancel_evt.is_set():
            return {'ok': False, 'msg': '探索启动已取消'}

        ok, mmsg = self.node.modes.claim('explore', 'EXPLORE',
                                         force=False)
        if not ok:
            return {'ok': False, 'msg': '✗ ' + mmsg}

        # 续建前置校验：车必须在已有图覆盖内
        if mode == 'continue' and not self._continued:
            ok, msg = self._check_start_in_map()
            if not ok:
                self.node.modes.release('explore')
                return {'ok': False, 'msg': '✗ 续建被拒：' + msg}
            ok, msg = self._deserialize(MAP_BASE)
            if not ok:
                self.node.modes.release('explore')
                return {'ok': False, 'msg': '✗ 续建载入失败：' + msg}
            self._continued = True

        if not self._client_ok():
            self.node.modes.release('explore')
            return {'ok': False, 'msg': '✗ navigate_to_pose 服务不在（nav2 栈没起？）'}

        with self.lock:
            if self.cancel_evt.is_set():
                self.node.modes.release('explore')
                return {'ok': False, 'msg': '探索启动已取消'}
            self.running = True
            self.st.update({'running': True, 'mode': mode, 'visited': 0,
                            'failed': 0, 'goal': None, 'saved_as': None,
                            'msg': '探索中（%s）' % ('续建' if mode == 'continue' else '当前建图')})
        try:
            threading.Thread(target=self._run, daemon=True).start()
        except RuntimeError as e:          # 线程耗尽：已 claim 但无线程走 finally
            self.node.modes.release('explore')
            with self.lock:
                self.running = False
            return {'ok': False, 'msg': '✗ 启动失败：%s' % e}
        return {'ok': True, 'msg': '探索已启动（%s）' % ('续建' if mode == 'continue' else '当前建图')}

    def cancel(self):
        """停止探索：置取消位（收尾由 _run 的 finally 负责）"""
        self.cancel_evt.set()
        with self.lock:
            handle = self._goal_handle
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
        self.node.publish_control('stop')
        self.node.set_cmd(0.0, 0.0)
        return {'ok': True, 'msg': '已请求停止'}

    def _check_mapping_mode(self):
        from rcl_interfaces.srv import GetParameters
        client = self.node.create_client(GetParameters, '/slam_toolbox/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2.0):
                return False, '无法确认 SLAM 建图模式，请检查 slam_toolbox'
            request = GetParameters.Request()
            request.names = ['mode']
            future = client.call_async(request)
            deadline = time.monotonic() + 3.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            if not future.done() or future.result() is None:
                return False, 'SLAM 模式查询超时'
            values = future.result().values
            if not values or values[0].string_value != 'mapping':
                return False, '当前是定位模式，不能扩图；请用 robot --nav2 --ekf 启动建图模式'
            return True, 'ok'
        finally:
            self.node.destroy_client(client)

    # ---------------- 续建 ----------------
    def _check_start_in_map(self):
        """车的起始位姿必须在已有图覆盖内，否则续建会生成连通孤岛"""
        pose = self.node.robot_pose(max_age=2.0)
        if pose is None:
            return False, '取不到定位 TF（slam_toolbox 没在跑？）'
        x, y = pose[0], pose[1]
        # 用当前 /map 的边界判断（此时已是已有图）
        m = getattr(self.node, 'raw_map_msg', None)
        if m is None:
            return False, '还没有 /map，无法判断起始位姿是否在图内'
        info = m.info
        if not (info.origin.position.x <= x <= info.origin.position.x + info.width * info.resolution
                and info.origin.position.y <= y <= info.origin.position.y + info.height * info.resolution):
            return False, ('车在 (%.2f, %.2f)，已在地图覆盖外 —— 请把车开回图内'
                           '，否则续建会生成连通孤岛' % (x, y))
        return True, 'ok'

    def _deserialize(self, prefix):
        from save_map import deserialize_posegraph
        started = time.monotonic()
        ok, msg = deserialize_posegraph(self.node, prefix, timeout=30.0, spin=False)
        if not ok:
            return ok, msg
        deadline = time.monotonic() + 5.0
        while self.node.raw_map_received_at <= started and time.monotonic() < deadline:
            if self.cancel_evt.is_set():
                return False, '已取消'
            time.sleep(0.1)
        if self.node.raw_map_received_at <= started:
            return False, '载入后未收到更新的原始 /map'
        return True, msg

    def _save_new_version(self):
        """存档不切：另存 robot_map_v2（已存在则递增），永不覆盖 robot_map.*"""
        from save_map import serialize_posegraph, save_pgm_yaml
        for i in range(2, 50):
            prefix = '%s_v%d' % (MAP_BASE, i)
            if not os.path.exists(prefix + '.posegraph'):
                ok, msg = serialize_posegraph(self.node, prefix, timeout=30.0, spin=False)
                if ok:
                    current = getattr(self.node, 'raw_map_msg', None)
                    if current is None:
                        return False, '缺少实时栅格图，存档不完整', None
                    save_pgm_yaml(current, prefix)
                return ok, msg, prefix
        return False, '版本号用尽（robot_map_v2.._v49 都已存在）', None

    # ---------------- nav2 目标 ----------------
    def _client_ok(self):
        if self._client is None:
            self._client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        return self._client.wait_for_server(3.0)

    def _send_goal(self, x, y):
        """发一个 NavigateToPose 目标；返回是否到达"""
        from geometry_msgs.msg import PoseStamped
        from action_msgs.msg import GoalStatus
        if self.cancel_evt.is_set() or self.node.robot_pose(max_age=2.0) is None:
            return False, '已取消或定位过期'
        with self.lock:
            if self.cancel_evt.is_set():
                return False, '已取消'
            self.node.publish_control('nav')
        g = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.header.frame_id = 'map'
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.w = 1.0                 # 朝向无所谓（yaw_goal_tolerance 3.15）
        g.pose = ps
        fut = self._client.send_goal_async(g)
        t0 = time.time()
        while time.time() - t0 < 5.0:
            time.sleep(0.1)
            if fut.done():
                break
        if not fut.done() or fut.result() is None:
            def cancel_late(done):
                handle = done.result()
                if handle is not None and handle.accepted:
                    handle.cancel_goal_async()
            fut.add_done_callback(cancel_late)
            return False, 'goal 未被接受'
        gh = fut.result()
        if not gh.accepted:
            return False, 'goal 被拒绝'
        with self.lock:
            self._goal_handle = gh
        rf = gh.get_result_async()
        t0 = time.time()
        while time.time() - t0 < GOAL_TIMEOUT:
            if self.cancel_evt.is_set():
                gh.cancel_goal_async()
                return False, '已取消'
            if self.node.robot_pose(max_age=2.0) is None:
                gh.cancel_goal_async()
                self.node.publish_control('stop')
                return False, '定位 TF 过期'
            time.sleep(0.1)
            if rf.done():
                break
        if not rf.done():
            gh.cancel_goal_async()
            return False, '目标超时'
        code = rf.result().status if rf.result() else -1
        pose = self.node.robot_pose(max_age=2.0)
        arrived = pose is not None and math.hypot(pose[0] - x, pose[1] - y) <= 0.30
        return code == GoalStatus.STATUS_SUCCEEDED and arrived, ('到达' if code == GoalStatus.STATUS_SUCCEEDED and arrived
                                                     else '导航返回状态 %s' % code)

    # ---------------- 主循环 ----------------
    def _run(self):
        node = self.node
        t_start = time.time()
        failed = 0
        excluded = []
        no_gain = 0
        first_map = getattr(node, 'raw_map_msg', None)
        initial_known = sum(v >= 0 for v in first_map.data) if first_map is not None else 0
        try:
            while not self.cancel_evt.is_set():
                if time.time() - t_start > TOTAL_CAP:
                    self._set(msg='已达总时长上限（%.0fs），自动停止' % TOTAL_CAP)
                    break

                m = getattr(node, 'raw_map_msg', None)
                if m is None:
                    self._set(msg='等待 /map ...')
                    time.sleep(0.2)
                    continue
                if time.monotonic() - node.raw_map_received_at > 5.0:
                    self._set(msg='原始 SLAM /map 超过 5 秒未更新，停止探索')
                    break

                grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
                clusters = detect_frontiers(grid)
                # 前沿点（世界坐标）供探索屏叠加；只取每簇代表点，最多 60 个
                ox, oy, res = (m.info.origin.position.x,
                               m.info.origin.position.y, m.info.resolution)
                if not clusters:
                    self._set(frontiers=0, frontier_pts=[])
                    self._set(msg='无前沿，探索完成')
                    break

                # 选最近的前沿（世界坐标距离）
                pose = node.robot_pose(max_age=2.0)
                if pose is None:
                    self._set(msg='定位 TF 过期，停止探索')
                    break
                goals = observation_goals(grid, clusters, res, (ox, oy), pose, excluded)
                self._set(frontiers=len(goals),
                          frontier_pts=[[round(g['x'], 3), round(g['y'], 3)] for g in goals[:60]])
                if not goals:
                    self._set(msg='无安全可达的新前沿：已过滤墙边缺口、噪点及已访问目标')
                    break
                gx, gy = goals[0]['x'], goals[0]['y']
                self._set(goal={'x': round(gx, 3), 'y': round(gy, 3)},
                          msg='前往安全观察点 (%.2f, %.2f)，路径约 %.1fm' % (gx, gy, goals[0]['path']))

                before = int((grid >= 0).sum())
                ok, why = self._send_goal(gx, gy)
                excluded.append((gx, gy))
                with self.lock:
                    self._goal_handle = None
                if self.cancel_evt.is_set():
                    break
                if ok:
                    failed = 0
                    self._set(visited=self.st['visited'] + 1, failed=0,
                              msg='已到达，继续找前沿')
                    deadline = time.monotonic() + 3.0
                    while not self.cancel_evt.is_set() and time.monotonic() < deadline:
                        time.sleep(0.1)
                    latest = getattr(node, 'raw_map_msg', None)
                    gain = sum(v >= 0 for v in latest.data) - before if latest is not None else 0
                    no_gain = no_gain + 1 if gain < 20 else 0
                    if no_gain >= 3:
                        self._set(msg='连续三个观察点未发现新区域，结束探索')
                        break
                else:
                    failed += 1
                    self._set(failed=failed, msg='目标失败：%s' % why)
                    if failed >= MAX_FAIL:
                        self._set(msg='连续 %d 次失败，停止探索' % failed)
                        break
        except Exception as e:
            self._set(msg='探索异常：%s' % e)
        finally:
            # 收尾（finally 语义，仿 patrol.py）：停车 + 释放控制权
            with self.lock:
                handle = self._goal_handle
                self._goal_handle = None
            if handle is not None:
                try:
                    handle.cancel_goal_async()
                except Exception:
                    pass
            node.set_cmd(0.0, 0.0)
            node.publish_control('stop')
            try:
                latest = getattr(node, 'raw_map_msg', None)
                new_known = sum(v >= 0 for v in latest.data) - initial_known if latest is not None else 0
                if new_known >= 20 and not self.cancel_evt.is_set():
                    ok, msg, prefix = self._save_new_version()
                    self._set(saved_as=prefix if ok else None,
                              msg=msg + ('（已存档，未激活）' if ok else '（保存失败）'))
                else:
                    self._set(msg=self.st['msg'] + '｜未保存（无新探索或已取消）')
            except Exception as exc:
                self._set(msg='探索结束，存档失败：%s' % exc)
            finally:
                self.node.modes.release('explore')
                with self.lock:
                    self.running = False
                    self.st['running'] = False
                    self.st['goal'] = None
