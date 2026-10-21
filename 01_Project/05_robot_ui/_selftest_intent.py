#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""intent.py 纯逻辑自测（不依赖 ROS / 板端；与 _selftest_patrol.py 同惯例）

覆盖：词表解析（含数字提取与词表外拒绝）、结构化解析、地点表查找
（精确/归一/子串/多候选/缺失）、stop 直通停止链与幂等豁免、
req_id 幂等、去抖、手动动作限速限时归零、PATROL 占用拒绝、
goto_place 命中发目标/未命中不发。

运行：python robot_ui/_selftest_intent.py（全部断言通过打印 ALL PASS）
"""
import os
import sys
import tempfile
import time
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_ui.intent import (IntentEngine, parse_structured, parse_text,
                             resolve_place)
from robot_ui.places import PlaceStore
from robot_ui.state_store import ModeMgr


class FakeExplore:
    def __init__(self):
        self.n_cancel = 0

    def cancel(self):
        self.n_cancel += 1
        return {'ok': True, 'msg': '探索已取消'}


class FakePatrol:
    def __init__(self):
        self.n_cancel = 0
        self.n_go = 0
        self.go_result = {'ok': True, 'msg': '巡逻出发'}

    def cancel(self):
        self.n_cancel += 1
        return {'ok': True, 'msg': '已取消'}

    def go(self):
        self.n_go += 1
        return dict(self.go_result)


class FakeNode:
    def __init__(self, places):
        self.modes = ModeMgr()
        self.patrol = FakePatrol()
        self.explore = FakeExplore()
        self.places = places
        self.cmds = []                 # (vx, wz)
        self.goals = []                # (x, y)
        self.controls = []
        self.n_wp_clear = 0
        self.n_clear_goal = 0
        self.n_goto_home = 0

    def set_cmd(self, vx, wz):
        self.cmds.append((round(vx, 4), round(wz, 4)))
        return vx, wz

    def set_goal(self, x=None, y=None, **kw):
        self.goals.append((x, y))
        return {'ok': True, 'msg': '目标已设', 'x': x, 'y': y}

    def goto_home(self):
        self.n_goto_home += 1
        return {'ok': True, 'msg': '回原点'}

    def clear_goal(self):
        self.n_clear_goal += 1
        return {'ok': True, 'msg': '目标已清除'}

    def wp_clear(self):
        self.n_wp_clear += 1
        return {'ok': True, 'msg': '队列已清空'}

    def publish_control(self, c):
        self.controls.append(c)

    def get_logger(self):
        class _L:
            def info(self, m):
                pass

            def warn(self, m):
                pass

            def error(self, m):
                pass
        return _L()


def make_places(tmp):
    prefix = os.path.join(tmp, 'robot_map')
    with open(prefix + '.yaml', 'w') as f:
        f.write('image: robot_map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    with open(prefix + '.pgm', 'wb') as f:
        f.write(b'P5\n2 2\n255\n' + bytes([254, 0, 0, 254]))
    ps = PlaceStore(prefix)
    ps.add('3D打印机', 5.13, -2.25)
    ps.add('充电位', 6.61, -1.02)
    ps.add('充电桩', 1.00, 1.00)      # 与「充电位」同前缀：制造多候选用例
    ps.add('卫生间', 2.71, 0.36)
    return ps


def main():
    tmp = tempfile.mkdtemp(prefix='intent-test-')
    places = make_places(tmp)
    node = FakeNode(places)
    eng = IntentEngine(node)

    # ---------- 1. 词表解析 ----------
    cases = {
        '停': 'stop', '停 止': 'stop', '快停': 'stop',
        '前进': 'forward', '向前走': 'forward',
        '后退': 'back', '倒退': 'back',
        '左转': 'left', '向左转': 'left',
        '右转': 'right', '往右': 'right',
        '回原点': 'home', '回家': 'home',
        '开始巡逻': 'patrol_start', '去巡逻': 'patrol_start',
        '停止巡逻': 'patrol_stop', '结束巡逻': 'patrol_stop',
    }
    for text, want in cases.items():
        got = parse_text(text)
        assert got and got['intent'] == want, '解析 %r → %r，期望 %r' % (text, got, want)
    # goto_place 模式
    got = parse_text('去充电位')
    assert got and got['intent'] == 'goto_place' and got['args']['name'] == '充电位', got
    got = parse_text('前往3D打印机')
    assert got and got['intent'] == 'goto_place' and got['args']['name'] == '3D打印机', got
    got = parse_text('去卫生间吧')
    assert got and got['intent'] == 'goto_place' and got['args']['name'] == '卫生间', got
    # 数字提取
    got = parse_text('前进2秒')
    assert got and got['intent'] == 'forward' and got['args']['duration'] == 2.0, got
    got = parse_text('左转90度')
    assert got and got['intent'] == 'left' and got['args']['angle'] == 90.0, got
    # 词表外一律 None（拒绝）
    for bad in ('讲个笑话', '飞起来', '', '   ', '去', '点外卖'):
        assert parse_text(bad) is None, '词表外 %r 应拒绝' % bad
    print('1. 词表解析 OK')

    # ---------- 2. 结构化解析 ----------
    ok = parse_structured({'intent': 'goto_place', 'args': {'name': '充电位'},
                           'req_id': 'esp32-000123'})
    assert ok and ok['intent'] == 'goto_place' and ok['req_id'] == 'esp32-000123', ok
    assert parse_structured({'intent': 'fly', 'req_id': 'x'}) is None
    assert parse_structured({'intent': 'stop', 'args': 'bad'}) is None
    assert parse_structured({'intent': 'stop', 'req_id': ''}) is None
    assert parse_structured({'intent': 'stop', 'req_id': 'x' * 81}) is None
    assert parse_structured('not-a-dict') is None
    assert parse_structured({'intent': 'stop'})['intent'] == 'stop'
    print('2. 结构化解析 OK')

    # ---------- 3. 地点表查找（robot_map.places.json 语义） ----------
    p, why = resolve_place(places, '充电位')
    assert p and p['x'] == 6.61 and p['y'] == -1.02, (p, why)
    p, _ = resolve_place(places, ' 充电 位 ')          # 归一
    assert p and p['name'] == '充电位', p
    p, _ = resolve_place(places, '打印机')              # 子串包含
    assert p and p['name'] == '3D打印机', p
    p, why = resolve_place(places, '充电')                # 多候选（充电位/充电桩）不猜
    assert p is None and '请说全名' in why, (p, why)
    p, why = resolve_place(places, '会议室')            # 未命中
    assert p is None and '不存在' in why, (p, why)
    print('3. 地点表查找 OK')

    # ---------- 4. stop 直通停止链 + 幂等豁免 + 去抖豁免 ----------
    for i in range(3):                                  # 连续 stop 必须每次都执行
        r = eng.execute('停止', source='voice')
        assert r['ok'] and r.get('duplicate') is None, r
    assert node.patrol.n_cancel == 3 and node.n_wp_clear == 3 and node.n_clear_goal == 3
    assert node.explore.n_cancel == 3, 'stop 链须连探索一起取消'
    assert node.controls == ['stop', 'stop', 'stop']
    print('4. stop 直通/豁免 OK')

    # ---------- 5. req_id 幂等 ----------
    node.goals.clear()
    r1 = eng.execute({'intent': 'goto_place', 'args': {'name': '充电位'},
                      'req_id': 'req-1'}, source='mqtt')
    assert r1['ok'] and node.goals == [(6.61, -1.02)], (r1, node.goals)
    r2 = eng.execute({'intent': 'goto_place', 'args': {'name': '充电位'},
                      'req_id': 'req-1'}, source='mqtt')
    assert r2['ok'] and r2.get('duplicate') is True, r2
    assert node.goals == [(6.61, -1.02)], '重复请求不得再次发目标'
    r3 = eng.execute({'intent': 'goto_place', 'args': {'name': '卫生间'},
                      'req_id': 'req-1'}, source='mqtt')
    assert not r3['ok'] and '已用于不同请求' in r3['msg'], r3
    print('5. req_id 幂等 OK')

    # ---------- 6. 去抖 ----------
    node.goals.clear()
    r1 = eng.execute({'intent': 'goto_place', 'args': {'name': '卫生间'},
                      'req_id': 'd-1'}, source='voice')
    assert r1['ok'], r1
    r2 = eng.execute({'intent': 'goto_place', 'args': {'name': '卫生间'},
                      'req_id': 'd-2'}, source='voice')   # 新 req_id 但窗口内同意图
    assert r2.get('duplicate') is True, r2
    assert node.goals == [(2.71, 0.36)], node.goals
    print('6. 去抖 OK')

    # ---------- 7. goto_place 未命中：拒绝且不发目标 ----------
    node.goals.clear()
    r = eng.execute({'intent': 'goto_place', 'args': {'name': '会议室'},
                     'req_id': 'g-1'}, source='voice')
    assert not r['ok'] and '不存在' in r['msg'] and node.goals == [], r
    print('7. 未命中拒绝 OK')

    # ---------- 8. 手动动作：限速 + 限时归零 ----------
    node.cmds.clear()
    r = eng.execute('前进0.3秒', source='voice')
    assert r['ok'], r
    time.sleep(0.8)                                     # 等动作线程跑完（0.3s + 收尾）
    assert node.cmds, '应有速度指令'
    assert all(abs(vx) <= 0.10 + 1e-6 and wz == 0.0 for vx, wz in node.cmds), node.cmds
    assert node.cmds[-1] == (0.0, 0.0), '动作结束必须归零：%r' % (node.cmds[-1],)
    assert eng.status()['moving'] is False
    # 时长上限钳位
    node.cmds.clear()
    r = eng.execute({'intent': 'forward', 'args': {'duration': 60.0},
                     'req_id': 'm-1'}, source='serial')
    assert r['ok'] and r['duration'] <= 3.0, r
    eng.execute('停止', source='voice')                 # 立即停（打断长动作）
    time.sleep(0.3)
    assert node.cmds[-1] == (0.0, 0.0)
    print('8. 手动限速限时 OK')

    # ---------- 9. PATROL 占用时手动拒绝 ----------
    assert node.modes.claim('patrol', 'PATROL', force=True)
    r = eng.execute('前进', source='voice')
    assert not r['ok'] and 'PATROL' in r['msg'], r
    node.modes.release('patrol')
    print('9. PATROL 互斥 OK')

    # ---------- 10. 巡逻意图与 home ----------
    r = eng.execute('开始巡逻', source='voice')
    assert r['ok'] and r['msg'] == '巡逻出发' and node.patrol.n_go == 1, r
    r = eng.execute('停止巡逻', source='voice')
    assert r['ok'] and node.patrol.n_cancel >= 1, r
    r = eng.execute('回原点', source='voice')
    assert r['ok'] and node.n_goto_home == 1, r
    print('10. 巡逻/home OK')

    print('ALL PASS')


if __name__ == '__main__':
    main()
