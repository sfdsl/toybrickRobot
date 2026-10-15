"""PatrolRunner 纯逻辑自测（Windows 上跑；板端不会执行本文件）

用途：patrol.py 顶部 import rclpy / nav2_msgs 在 Windows 上不可用，
本脚本先用 sys.modules 塞入最小 stub（假 ActionClient / 假 goal handle），
再实例化 PatrolRunner（node 用假节点：modes=ModeMgr()、set_cmd 记录器、
可编程 precheck），验证：
  load 校验 / 逐段预检拒绝 / go 的 claim 互斥 / 服务不在分支 /
  失败跳过 / skip 打断导航与停留 / cancel 中断后 release('patrol') /
  pause 暂停门 / 重复 go 与运行中 load 拒绝。

运行：python robot_ui/_selftest_patrol.py（全部断言通过打印 ALL PASS）
"""
import os
import sys
import time
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ---------------- ROS 最小 stub（只覆盖 patrol.py 用到的面） ----------------
_rclpy = types.ModuleType('rclpy')
_rclpy.ok = lambda: True
_action = types.ModuleType('rclpy.action')


class _Future:
    """假 future：_done/_res 手工控制"""

    def __init__(self, res=None, done=True):
        self._res, self._done = res, done

    def done(self):
        return self._done

    def result(self):
        return self._res


class FakeGH:
    """假 goal handle：mode='done' 立即成功；'block' 永不完成（只考察 cancel/skip 分支）"""
    n_cancel = 0                       # 类级计数：全流程取消次数

    def __init__(self, mode, accepted=True):
        self.mode = mode
        self.accepted = accepted

    def cancel_goal_async(self):
        FakeGH.n_cancel += 1
        return _Future(res=self)

    def get_result_async(self):
        if self.mode == 'block':
            return _Future(done=False)             # 永不完成
        return _Future(res=types.SimpleNamespace(status=4))   # 4 = SUCCEEDED


class FakeActionClient:
    server_up = True
    script = []                        # 每点行为队列：'done' / 'block' / 'reject'

    def __init__(self, node, atype, aname):
        self.node = node

    def wait_for_server(self, timeout):
        return FakeActionClient.server_up

    def send_goal_async(self, goal):
        mode = FakeActionClient.script.pop(0) if FakeActionClient.script else 'done'
        if mode == 'done':
            self.node.pose = (goal.pose.pose.position.x, goal.pose.pose.position.y, 0.0)
        gh = FakeGH(mode, accepted=(mode != 'reject'))
        return _Future(res=gh)


_action.ActionClient = FakeActionClient
_rclpy.action = _action
sys.modules['rclpy'] = _rclpy
sys.modules['rclpy.action'] = _action

_geo = types.ModuleType('geometry_msgs')
_geomsg = types.ModuleType('geometry_msgs.msg')


class PoseStamped:
    def __init__(self):
        self.header = types.SimpleNamespace(frame_id='', stamp=None)
        self.pose = types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0),
            orientation=types.SimpleNamespace(w=0.0))


_geomsg.PoseStamped = PoseStamped
_geo.msg = _geomsg
sys.modules['geometry_msgs'] = _geo
sys.modules['geometry_msgs.msg'] = _geomsg

_nav = types.ModuleType('nav2_msgs')
_navact = types.ModuleType('nav2_msgs.action')


class NavigateToPose:
    class Goal:
        def __init__(self):
            self.pose = PoseStamped()


_navact.NavigateToPose = NavigateToPose
_nav.action = _navact
sys.modules['nav2_msgs'] = _nav
sys.modules['nav2_msgs.action'] = _navact

# stub 就位后再导入被测模块
import robot_ui.patrol as patrol_mod              # noqa: E402  （monkeypatch TOTAL_CAP 用）
from robot_ui.patrol import PatrolRunner          # noqa: E402
from robot_ui.state_store import ModeMgr          # noqa: E402


# ---------------- 假节点 ----------------
class FakePrecheck:
    """可编程预检：mode='ok' 全通过（路径 1.5m）；'fail' 全拒绝；fn 自定义"""

    def __init__(self, mode='ok'):
        self.mode = mode
        self.calls = []

    def check(self, start, goal):
        self.calls.append((start, goal))
        if self.mode == 'fail':
            return {'ok': False, 'msg': '规划失败：被墙挡住'}
        return {'ok': True, 'msg': 'ok', 'dist': 1.5}


class FakeNode:
    def __init__(self, precheck_mode='ok'):
        self.modes = ModeMgr()
        self.precheck = FakePrecheck(precheck_mode)
        self.cmds = []
        self.pose = (0.0, 0.0, 0.0)
        self.tf_fresh = True

    def set_cmd(self, vx, wz):
        self.cmds.append((vx, wz))

    def robot_pose(self, max_age=None):
        if max_age is not None and not self.tf_fresh:
            return None
        return self.pose

    def get_clock(self):
        class _C:
            def now(self):
                class _T:
                    def to_msg(self):
                        return 0
                return _T()
        return _C()

    def get_logger(self):
        class _L:
            def info(self, m):
                pass

            def warn(self, m):
                pass

            def error(self, m):
                pass
        return _L()


def wait_done(p, timeout=8.0):
    """等巡逻线程自然收尾"""
    t0 = time.time()
    while p.status()['running'] and time.time() - t0 < timeout:
        time.sleep(0.05)


def pts2(dwell=0.0):
    return [{'x': 1.0, 'y': 0.0, 'dwell': dwell, 'name': 'A'},
            {'x': 2.0, 'y': 0.0, 'dwell': dwell, 'name': 'B'}]


results = []


def check(name, cond, extra=''):
    results.append((name, cond, extra))
    print('%-28s %s %s' % (name, 'PASS' if cond else 'FAIL', extra))
    assert cond, name


# ---------------- T1 load 校验 ----------------
n = FakeNode()
p = PatrolRunner(n)
r = p.load([{'x': 'abc', 'y': 0.0}], False)
check('T1a 非数字坐标拒绝', not r['ok'] and '不是数字' in r['msg'], r['msg'])
r = p.load([{'x': 1.0, 'y': 2.0, 'dwell': 99.0}, {'x': 2.0, 'y': 2.0}], False)
check('T1b dwell 夹取 0~60/缺省3', r['ok'] and p.pts[0]['dwell'] == 60.0
      and p.pts[1]['dwell'] == 3.0, str([q['dwell'] for q in p.pts]))
check('T1c 缺省点名', p.pts[1]['name'] == '点2', p.pts[1]['name'])
r = p.points()
check('T1d points() 回显', r['ok'] and len(r['pts']) == 2 and r['loop'] is False)

# ---------------- T2 逐段预检拒绝（第 2 点失败 → 整批拒绝） ----------------
n = FakeNode()
p = PatrolRunner(n)
n.precheck.check = lambda s, g: ({'ok': False, 'msg': '被墙挡住'}
                                 if g[0] > 1.5 else {'ok': True, 'msg': 'ok', 'dist': 1.0})
r = p.load(pts2(), False)
check('T2a 预检失败整批拒绝', not r['ok'] and '第 2 点' in r['msg'] and '被墙挡住' in r['msg'], r['msg'])
check('T2b 拒绝后不写入 pts', p.pts == [], str(p.pts))   # 新 runner，整批拒绝后应为空
n2 = FakeNode(precheck_mode='fail')
p2 = PatrolRunner(n2)
r = p2.load(pts2(), False)
check('T2c 全失败报第 1 点', not r['ok'] and '第 1 点' in r['msg'], r['msg'])

# ---------------- T3 go 明确接管 TELEOP ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
ok, _ = n.modes.claim('tester', 'TELEOP')
assert ok
r = p.go()
check('T3a 巡逻接管 TELEOP', r['ok'], r['msg'])
wait_done(p)
n.modes.release('tester')
check('T3b 完成后释放模式', n.modes.snapshot()['mode'] == 'IDLE')

# ---------------- T4 navigate_to_pose 服务不在 ----------------
FakeActionClient.server_up = False
r = p.go()
check('T4a 服务不在拒绝', not r['ok'] and 'navigate_to_pose 服务不在' in r['msg'], r['msg'])
check('T4b 失败后释放 PATROL', n.modes.snapshot()['mode'] == 'IDLE')
FakeActionClient.server_up = True

# ---------------- T5 失败跳过（reject → failed，继续下一点） ----------------
FakeActionClient.script = ['reject', 'done']
FakeGH.n_cancel = 0
r = p.go()
check('T5a 出发应答', r['ok'], r['msg'])
wait_done(p)
st = p.status()
check('T5c 逐点状态', st['per'] == ['failed', 'done'], str(st['per']))
check('T5d 结束语（失败 1 点）', st['msg'] == '已完成（失败 1 点）', st['msg'])
check('T5e 收尾释放 PATROL', n.modes.snapshot()['mode'] == 'IDLE')
check('T5f 收尾停车', n.cmds[-1] == (0.0, 0.0), str(n.cmds[-1]))
check('T5g total/eta 归零', st['total'] == 2 and st['eta'] == 0.0)

# ---------------- T6 skip 打断导航（block → skipped → 下一点 done） ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
FakeActionClient.script = ['block', 'done']
r = p.go()
assert r['ok'], r['msg']
time.sleep(0.5)
r = p.skip()
check('T6a skip 应答', r['ok'])
wait_done(p)
st = p.status()
check('T6b 逐点状态', st['per'] == ['skipped', 'done'], str(st['per']))
check('T6c 结束语', st['msg'] == '已完成', st['msg'])
check('T6d 释放 PATROL', n.modes.snapshot()['mode'] == 'IDLE')

# ---------------- T7 skip 打断停留（done + dwell=5 → 提前走） ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load([{'x': 1.0, 'y': 0.0, 'dwell': 5.0, 'name': 'A'}], False)
FakeActionClient.script = ['done']
t0 = time.time()
r = p.go()
assert r['ok'], r['msg']
time.sleep(0.6)                    # 已进停留
p.skip()
wait_done(p)
check('T7a 停留被跳过提前结束', time.time() - t0 < 3.0, '%.1fs' % (time.time() - t0))
check('T7b 该点仍记 done', p.status()['per'] == ['done'], str(p.status()['per']))

# ---------------- T8 cancel 中断（收尾 release + 停车） ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
FakeActionClient.script = ['block']
r = p.go()
assert r['ok'], r['msg']
time.sleep(0.5)
r = p.load(pts2(), False)          # 运行中装载应被拒
check('T8a 运行中 load 拒绝', not r['ok'] and '取消' in r['msg'], r['msg'])
r = p.cancel()
check('T8b cancel 应答', r['ok'], r['msg'])
wait_done(p)
st = p.status()
check('T8c 结束语 已取消', st['msg'] == '已取消', st['msg'])
check('T8d running=False', st['running'] is False)
check('T8e 释放 PATROL', n.modes.snapshot()['mode'] == 'IDLE')
check('T8f 有向 nav2 发取消', FakeGH.n_cancel >= 1, 'n_cancel=%d' % FakeGH.n_cancel)
check('T8g 收尾停车', n.cmds[-1] == (0.0, 0.0))

# ---------------- T9 pause 暂停门（暂停后不出发下一点，resume 放行） ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
FakeActionClient.script = ['block', 'done']
r = p.go()
assert r['ok'], r['msg']
r_dup = p.go()                     # 第 1 点 block 挂起中，此刻必然 running=True
check('T9a0 重复 go 拒绝', not r_dup['ok'] and '执行中' in r_dup['msg'], r_dup['msg'])
time.sleep(0.4)
check('T9a pause 应答', p.pause()['ok'])
time.sleep(0.2)
p.skip()                           # 结束第 1 点（导航被取消 → skipped）
time.sleep(0.8)
st = p.status()
check('T9b 暂停中停在下一点门口', st['paused'] and st['cur'] == 0
      and st['per'] == ['skipped', 'pending'], str((st['cur'], st['per'])))
check('T9c resume 应答', p.resume()['ok'])
wait_done(p)
st = p.status()
check('T9d 恢复后走完', st['per'] == ['skipped', 'done'], str(st['per']))
check('T9e paused 复位', st['paused'] is False)

# ---------------- T10 空点 / 未运行时的控制入口 ----------------
n = FakeNode()
p = PatrolRunner(n)
r = p.go()
check('T10a 空点 go 拒绝', not r['ok'] and '为空' in r['msg'], r['msg'])
check('T10b 未运行 skip 拒绝', not p.skip()['ok'])
check('T10c 未运行 pause 拒绝', not p.pause()['ok'])
check('T10d 未运行 resume 拒绝', not p.resume()['ok'])
check('T10e 未运行 cancel 允许(幂等)', p.cancel()['ok'])
check('T10f 状态键齐全', set(p.status()) >= {'cur', 'total', 'per', 'elapsed',
                                             'eta', 'msg', 'running', 'paused', 'loop'})

# ---------------- T11 TOTAL_CAP 总时长上限（loop=True 防无限圈） ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), True)                       # loop=True 双点场景
_orig_cap = patrol_mod.TOTAL_CAP
patrol_mod.TOTAL_CAP = 0.05                # monkeypatch 成极小值，触发圈头上限
try:
    r = p.go()
    assert r['ok'], r['msg']
    wait_done(p)
finally:
    patrol_mod.TOTAL_CAP = _orig_cap
st = p.status()
check('T11a 总时长上限自动终止', st['running'] is False and '上限' in st['msg'], st['msg'])
check('T11b 上限收尾释放 PATROL', n.modes.snapshot()['mode'] == 'IDLE')
check('T11c 上限收尾停车', n.cmds[-1] == (0.0, 0.0), str(n.cmds[-1]))

# ---------------- T12 load TOCTOU（预检窗口内 go() 已启动 → 写入前再查拒绝） ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
_orig_pts = [dict(q) for q in p.pts]


def _slow_check(start, goal):
    """模拟预检耗时窗口里并发 go() 已把 running 置真"""
    p.running = True
    return {'ok': True, 'msg': 'ok', 'dist': 1.0}


p.node.precheck.check = _slow_check
r = p.load([{'x': 9.0, 'y': 9.0, 'dwell': 0.0, 'name': 'X'}], False)
check('T12a 写入窗内 load 拒绝', not r['ok'] and '禁止装载' in r['msg'], r['msg'])
check('T12b pts 未被覆写', p.pts == _orig_pts, str(p.pts))
p.running = False                          # 还原假标志（未起真线程，无 claim 需释放）

# ---------------- T13 定位失效时拒绝出发 ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
n.tf_fresh = False
r = p.go()
check('T13a 过期 TF 拒绝出发', not r['ok'] and '定位 TF' in r['msg'], r['msg'])
check('T13b 未认领巡逻模式', n.modes.snapshot()['mode'] == 'IDLE')

# ---------------- T14 Nav2 假报成功但车未到站 ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
FakeActionClient.script = ['false_success', 'done']
r = p.go()
assert r['ok'], r['msg']
wait_done(p)
st = p.status()
check('T14a 假成功标失败', st['per'] == ['failed', 'pending'], str(st['per']))
check('T14b 后续航点未下发', FakeActionClient.script == ['done'])
check('T14c 结束语提示定位异常', '定位异常' in st['msg'], st['msg'])

# ---------------- T15 目标执行中 TF 停更 ----------------
n = FakeNode()
p = PatrolRunner(n)
p.load(pts2(), False)
FakeActionClient.script = ['block', 'done']
FakeGH.n_cancel = 0
r = p.go()
assert r['ok'], r['msg']
time.sleep(0.3)
n.tf_fresh = False
wait_done(p)
st = p.status()
check('T15a 中途定位失效停巡', st['per'] == ['failed', 'pending'], str(st['per']))
check('T15b 已取消 Nav2 目标', FakeGH.n_cancel >= 1)

print('\n%d/%d 项断言全部通过：ALL PASS' % (len(results), len(results)))
