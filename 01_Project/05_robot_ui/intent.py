#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""三源共用意图词表 + 解析 + 执行入口（浏览器语音 / MQTT 远程 / ESP32 串口）

设计（《05_实施清单/P3-语音与远程入口.md》）：
  三源一口：语音文本、MQTT 意图 JSON、串口意图 JSON 先归一为意图对象，
  再由本执行器翻译成 goal / manual / stop，走与 /task/request 完全相同的
  语义（ModeMgr 认领、req_id 幂等、PlaceStore 地图版本校验）。
  **任何新入口不得直发 /cmd_vel**——手动速度一律经 node.set_cmd()
  （/cmd_vel_manual → control_mux → collision_monitor → /cmd_vel）。

安全口径（卡-05 / 路线图 §1）：
  · 词表外话语拒绝且不触发动作
  · stop 最高优先、免鉴权、幂等豁免（重复 stop 重复停），直通本地停止链
  · 手动类限速 ≤0.10 m/s、限时自动归零（另有 PhoneTeleop.CMD_TIMEOUT=0.6s 看门狗兜底）
  · 「去<地点>」只查 robot_map.places.json（PlaceStore），未命中/换图即拒绝，
    语音与大模型不得凭空造坐标
  · 同一 req_id 300s 内不重复执行；去抖窗口内同意图不重复执行（stop 豁免）
"""
import math
import re
import threading
import time
import unicodedata

# ---------------- 安全参数（改这里可全局调，禁止超过下列上限） ----------------
MANUAL_VX = 0.10            # 手动前进/后退限速 m/s（卡-05：语音动作要限速）
MANUAL_WZ = 0.50            # 手动左/右转限速 rad/s（须 > 底盘死区 0.15，P9 实测）
MANUAL_DURATION = 1.5       # 默认动作时长 s
MANUAL_DURATION_MAX = 3.0   # 线性动作时长上限 s
ROT_DURATION_MAX = 5.0      # 旋转动作时长上限 s（90°@0.5rad/s ≈ 3.1s）
DEBOUNCE_S = 1.0            # 同意图去抖窗口 s（stop 豁免）
IDEMPOTENCY_WINDOW_S = 300.0  # req_id 幂等窗口 s（与 routes_task 同口径）
MOTION_HZ = 10.0            # 手动动作续发频率（看门狗 0.6s，10Hz 续发不断流）

# ---------------- 有限意图词表（需人工确认；词表外一律拒绝） ----------------
VOCAB = ('stop', 'forward', 'back', 'left', 'right',
         'home', 'goto_place', 'patrol_start', 'patrol_stop')

# 文本 → 意图（长词在前；goto_place 用动词+地名模式兜底，排最后）
_TEXT_RULES = (
    (r'^(停止巡逻|结束巡逻|巡逻停止|别巡逻)', 'patrol_stop'),
    (r'^(开始巡逻|去巡逻|巡逻开始|开始巡检)', 'patrol_start'),
    (r'^(停|停止|停下|快停|停止一切|别动|立定)', 'stop'),
    (r'^(前进|向前|往前|往前走|向前走)', 'forward'),
    (r'^(后退|倒退|向后|往后|往后退|向后退)', 'back'),
    (r'^(左转|向左转|往左转|向左|往左)', 'left'),
    (r'^(右转|向右转|往右转|向右|往右)', 'right'),
    (r'^(回原点|回起点|回家|回到原点|回到起点)', 'home'),
)
_GOTO_RE = re.compile(r'^(?:去|到|前往|回)(.+)$')
_DUR_RE = re.compile(r'(\d+(?:\.\d+)?)\s*(?:秒|s)', re.IGNORECASE)
_ANG_RE = re.compile(r'(\d+(?:\.\d+)?)\s*度')
_PARTICLES = ('一下', '吧', '啊', '呢', '呀', '点')   # 口语尾巴，剥掉


def _norm_text(text):
    """文本归一：全角→半角、去空白与常见标点、口语尾巴暂不剥（留给名称解析）"""
    t = unicodedata.normalize('NFKC', str(text or ''))
    t = re.sub(r'[\s，。！？、,.!?;；:：]+', '', t)
    return t


def _norm_name(name):
    """地点名归一：全角→半角、去空白与标点、去口语尾巴"""
    n = unicodedata.normalize('NFKC', str(name or ''))
    n = re.sub(r'[\s，。！？、,.!?;；:：]+', '', n)
    for p in _PARTICLES:
        if n.endswith(p) and len(n) > len(p):
            n = n[:-len(p)]
    return n


def parse_text(text):
    """语音文本 → 意图 dict；词表外返回 None（拒绝，不触发动作）

    返回 {'intent', 'args', 'text'}；数字提取：N秒 → duration，N度 → angle。
    「去<地点名>」走 goto_place，名称交 resolve_place() 去地点表查。
    """
    t = _norm_text(text)
    if not t:
        return None
    args = {}
    m = _DUR_RE.search(text)
    if m:
        args['duration'] = float(m.group(1))
    m = _ANG_RE.search(text)
    if m:
        args['angle'] = float(m.group(1))
    for pat, intent in _TEXT_RULES:
        if re.match(pat, t):
            return {'intent': intent, 'args': args, 'text': str(text)}
    m = _GOTO_RE.match(t)
    if m:
        name = _norm_name(m.group(1))
        if name:
            return {'intent': 'goto_place', 'args': dict(args, name=name),
                    'text': str(text)}
    return None


def parse_structured(obj):
    """结构化意图（MQTT/串口 JSON）→ 意图 dict；非法返回 None

    期望 {'intent': 词表枚举, 'args': {...}, 'req_id': '...'}。
    intent 不在词表 / args 类型不对 → None。
    """
    if not isinstance(obj, dict):
        return None
    intent = obj.get('intent')
    if intent not in VOCAB:
        return None
    args = obj.get('args')
    if args is None:
        args = {}
    if not isinstance(args, dict):
        return None
    out = {'intent': intent, 'args': dict(args), 'text': str(obj.get('text') or '')}
    req_id = obj.get('req_id')
    if req_id is not None:
        if not isinstance(req_id, str) or not req_id or len(req_id) > 80:
            return None
        out['req_id'] = req_id
    return out


def resolve_place(places, name):
    """查 PlaceStore（robot_map.places.json）；返回 (地点dict|None, 原因)

    查找顺序：精确 → 全半角/空白归一 → 子串包含（「去3D打印机」取「3D打印机」）。
    多候选不猜测；换图后 PlaceStore 会抛 PlaceError，一并转成拒绝原因。
    """
    from robot_ui.places import PlaceError
    try:
        data = places.list()
    except (PlaceError, OSError, ValueError) as e:
        return None, str(e)
    table = data.get('places') or {}
    if not table:
        return None, '地点表为空，先在航点页保存地点'
    key = str(name or '')
    if key in table:
        return dict(table[key], name=key), ''
    nk = _norm_name(key)
    normed = {_norm_name(k): k for k in table}
    if nk in normed:
        k = normed[nk]
        return dict(table[k], name=k), ''
    cands = [k for k in table if nk and (nk in _norm_name(k) or _norm_name(k) in nk)]
    if len(cands) == 1:
        k = cands[0]
        return dict(table[k], name=k), ''
    if len(cands) > 1:
        return None, '有 %d 个相近地点（%s），请说全名' % (len(cands), '、'.join(cands[:4]))
    return None, '地点不存在：%s' % key


class IntentEngine:
    """意图执行器（进程内单实例，挂在 PhoneTeleop 旁；仿 PatrolRunner 生命周期）

    · 不自建 executor/spin：所有 node 调用由 app.py 的 spin 线程驱动
    · 手动动作用后台线程按 MOTION_HZ 续发 set_cmd，到时自动归零
    · req_id 幂等 + 同意图去抖（stop 全豁免）
    """

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self._seen = {}            # (source, req_id) -> (t, signature)
        self._debounce = {}        # (source, sig) -> t
        self._motion_stop = None   # 手动动作取消 Event；None = 无动作
        self._motion_thread = None
        self.last = None           # 最近一次执行结果（state() 用）
        self.n_exec = 0
        self.n_reject = 0

    # ---------- 内部工具 ----------
    @staticmethod
    def _signature(it):
        return (it.get('intent'), tuple(sorted((k, str(v)) for k, v in
                                               (it.get('args') or {}).items())))

    def _log(self, level, msg):
        try:
            getattr(self.node.get_logger(), level)(msg)
        except Exception:
            print('[intent] %s: %s' % (level, msg), flush=True)

    def _stop_motion(self):
        ev = self._motion_stop
        if ev is not None:
            ev.set()

    def _start_motion(self, vx, wz, duration):
        """限时手动动作：按 MOTION_HZ 续发（看门狗 0.6s 会把断流变零速），到时归零"""
        self._stop_motion()
        ev = threading.Event()
        self._motion_stop = ev

        def _run():
            t_end = time.time() + duration
            try:
                while not ev.is_set() and time.time() < t_end:
                    self.node.set_cmd(vx, wz)
                    time.sleep(1.0 / MOTION_HZ)
            finally:
                self.node.set_cmd(0.0, 0.0)   # 动作结束/被打断都必须归零
                if self._motion_stop is ev:
                    self._motion_stop = None
        th = threading.Thread(target=_run, daemon=True)
        self._motion_thread = th
        th.start()

    # ---------- 对外入口 ----------
    def parse(self, payload):
        """payload：str（语音文本）或 dict（结构化 JSON）→ 意图 dict | None"""
        if isinstance(payload, dict):
            return parse_structured(payload)
        return parse_text(payload)

    def execute(self, payload, source='voice', req_id=None):
        """解析并执行；永不抛异常。返回 {'ok', 'msg', ...}（词表外 ok=False）"""
        try:
            return self._execute_inner(payload, source, req_id)
        except Exception as e:                       # 防御：任何执行器内部错误都不外溢
            self._log('error', '意图执行异常：%r' % e)
            with self.lock:
                self.last = {'ok': False, 'msg': '执行异常：%s' % e, 'source': source}
                self.n_reject += 1
            return dict(self.last)

    def _execute_inner(self, payload, source, req_id):
        it = self.parse(payload)
        if it is None:
            with self.lock:
                self.n_reject += 1
                self.last = {'ok': False, 'msg': '词表外指令：已拒绝，未执行任何动作',
                             'source': source}
            self._log('warn', '拒绝（词表外）source=%s payload=%r' % (source, payload))
            return dict(self.last)
        if req_id:
            it['req_id'] = req_id
        intent, args = it['intent'], (it.get('args') or {})

        # ---- stop 最高优先：免鉴权、幂等豁免、去抖豁免，直通本地停止链 ----
        if intent == 'stop':
            self._stop_motion()
            self.node.explore.cancel()
            self.node.patrol.cancel()
            self.node.wp_clear()
            self.node.clear_goal()
            self.node.publish_control('stop')
            res = {'ok': True, 'msg': '已停止（巡逻/导航/手动动作全部清除）',
                   'intent': 'stop', 'source': source}
            with self.lock:
                self.last = res
                self.n_exec += 1
            self._log('info', '停止 source=%s' % source)
            return dict(res)

        # ---- req_id 幂等（同 routes_task 口径：同 ID 同参返回 duplicate，异参拒绝）----
        rid = it.get('req_id')
        sig = self._signature(it)
        if rid:
            key = (source, rid)
            with self.lock:
                now = time.monotonic()
                for k, (t, _) in list(self._seen.items()):
                    if now - t > IDEMPOTENCY_WINDOW_S:
                        del self._seen[k]
                if key in self._seen:
                    if self._seen[key][1] == sig:
                        return {'ok': True, 'duplicate': True,
                                'msg': '重复请求已忽略', 'intent': intent}
                    self.n_reject += 1
                    return {'ok': False, 'msg': 'request_id 已用于不同请求'}

        # ---- 去抖：窗口内同意图同参数不重复执行（stop 已豁免）----
        dkey = (source, sig)
        now = time.monotonic()
        with self.lock:
            last_t = self._debounce.get(dkey)
            if last_t is not None and now - last_t < DEBOUNCE_S:
                return {'ok': True, 'duplicate': True,
                        'msg': '去抖窗口内重复指令已忽略', 'intent': intent}
            self._debounce[dkey] = now

        if rid:
            with self.lock:
                self._seen[(source, rid)] = (time.monotonic(), sig)

        # ---- 执行 ----
        res = self._dispatch(intent, args, source)
        res['intent'] = intent
        res['source'] = source
        with self.lock:
            self.last = res
            if res.get('ok'):
                self.n_exec += 1
            else:
                self.n_reject += 1
        self._log('info' if res.get('ok') else 'warn',
                  '%s source=%s → %s' % (intent, source, res.get('msg')))
        return dict(res)

    def _dispatch(self, intent, args, source):
        node = self.node
        # 手动类：限时限速；认领 TELEOP（对 NAV 自动抢占，仅 PATROL 拒绝——与 routes_task 同）
        if intent in ('forward', 'back', 'left', 'right'):
            ok, mmsg = node.modes.claim(source, 'TELEOP', force=True, force_unless='PATROL')
            if not ok:
                return {'ok': False, 'msg': mmsg}
            try:
                dur = float(args.get('duration', MANUAL_DURATION))
            except (TypeError, ValueError):
                return {'ok': False, 'msg': 'duration 无效'}
            if not math.isfinite(dur) or dur <= 0:
                return {'ok': False, 'msg': 'duration 无效'}
            if intent in ('forward', 'back'):
                dur = min(dur, MANUAL_DURATION_MAX)
                vx = MANUAL_VX if intent == 'forward' else -MANUAL_VX
                wz = 0.0
            else:
                dur = min(dur, ROT_DURATION_MAX)
                if 'angle' in args:                 # N度 → 时长（固定角速度）
                    try:
                        ang = float(args['angle'])
                    except (TypeError, ValueError):
                        return {'ok': False, 'msg': 'angle 无效'}
                    if not math.isfinite(ang) or ang <= 0:
                        return {'ok': False, 'msg': 'angle 无效'}
                    dur = min(max(ang / MANUAL_WZ * math.pi / 180.0, 0.2),
                              ROT_DURATION_MAX)
                vx = 0.0
                wz = MANUAL_WZ if intent == 'left' else -MANUAL_WZ
            self._start_motion(vx, wz, dur)
            return {'ok': True, 'msg': '%s %.1fs（限速 %.2f m/s / %.2f rad/s）'
                    % ({'forward': '前进', 'back': '后退', 'left': '左转',
                        'right': '右转'}[intent], dur, vx, wz),
                    'duration': round(dur, 2)}

        # 导航类：NAV 认领（非 web 源不可强抢人工，PATROL 不可抢——同 routes_task）
        if intent in ('home', 'goto_place'):
            ok, mmsg = node.modes.claim(source, 'NAV',
                                        force=(source == 'web'), force_unless='PATROL')
            if not ok:
                return {'ok': False, 'msg': mmsg}
            if intent == 'home':
                return node.goto_home()
            name = (args.get('name') or '')
            place, why = resolve_place(node.places, name)
            if place is None:
                node.modes.release(source)         # 认领了但没出发，归还控制权
                return {'ok': False, 'msg': why}
            return node.set_goal(x=place['x'], y=place['y'])

        # 巡逻类：复用 PatrolRunner（内部自带 PATROL 认领与释放）
        if intent == 'patrol_start':
            return node.patrol.go()
        if intent == 'patrol_stop':
            return node.patrol.cancel()
        return {'ok': False, 'msg': '未知意图：%s' % intent}

    # ---------- 状态（state() / 语音屏用） ----------
    def status(self):
        with self.lock:
            return {
                'vocab': list(VOCAB),
                'last': dict(self.last) if self.last else None,
                'n_exec': self.n_exec,
                'n_reject': self.n_reject,
                'moving': self._motion_stop is not None,
            }
