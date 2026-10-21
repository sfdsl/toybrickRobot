#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mqtt_bridge.py — MQTT 遥测/任务桥（P3 / 卡-04）

把小车状态上云、把云端任务收回统一任务入口。**绝不直发 /cmd_vel**（卡-04 禁止行为）：
任务一律经 localhost HTTP 转发到 robot_ui 的 `/task/request` / `/voice/intent`
（与网页同一鉴权、同一 ModeMgr、同一 request_id 幂等），速度仍走
cmd_vel_manual → control_mux → collision_monitor → /cmd_vel 安全链。

消息契约（JSON；dev=设备ID、ts=epoch秒、seq=单调序列号，卡-04 三要素）：
  上行  robot/<dev>/telemetry  {dev,ts,seq,pose:{x,y,yaw},vel:{vx,wz},
                               battery:{voltage,pct},mode:{mode,holder},goal}
  上行  robot/<dev>/result     {dev,ts,seq,req_id,ok,msg}
  下行  robot/<dev>/task       {dev,ts,seq,req_id,action,args,on_disconnect}
        action ∈ stop | goal | intent | manual | heartbeat
          stop      全停（显式命令：取消巡逻/导航/手动）
          goal      args {x,y,map_id?} 或 {name}（地点名走意图层查 places.json）
          intent    args {intent, args}（词表内结构化意图，走 /voice/intent）
          manual    args {vx,wz,lease_ms?}（限时手动，心跳续租）
          heartbeat args 可空（仅续租，不改速度）
        on_disconnect ∈ continue | stop（仅对 goal 生效；缺省 continue）
  在线  robot/<dev>/presence    online|offline（retain + 遗嘱；重连后重发 online——
        红队坑「重连成功后未重新发布遗嘱消息」的对策）

心跳租约（路线图 §1：600ms 心跳停车只作用于远程租约）：
  · remote manual 每条 task（manual/heartbeat）续租 lease_ms（缺省 600，上限 5000）
  · 租约期内桥按 ≤10 Hz 续发 manual 到 /task/request（CMD_TIMEOUT=0.6s 看门狗不断流）
  · 租约到期（云端心跳停）→ 立即补发一次 vx=0,wz=0（600ms 内触发停车，卡-04 验收）
  · MQTT 断链 → 同上补零；goal 的 on_disconnect=stop 时再补 action=stop
  · 若本地已接管（/state 的 mode.holder 不再是 mqtt）→ 静默丢弃租约，不抢、不补零

用法（云端 Broker + TLS，卡-04 默认口径）：
  python3 mqtt_bridge.py --host <broker> --port 8883 --device car01 \\
      --tls-ca ca.pem [--tls-cert client.pem --tls-key client.key] \\
      [--user u --pass-file ~/.mqtt_pass] [--ui http://127.0.0.1:8080]
  python3 mqtt_bridge.py --selftest        # 纯逻辑自测（无 broker/ROS 依赖）
  python3 mqtt_bridge.py --host 127.0.0.1 --no-tls   # 本地 mosquitto 调试（警告）

凭证口径（卡-04「账号密码/证书走环境变量或密钥文件」）：
  账号 MQTT_USER / 密码 MQTT_PASS 环境变量优先；或 --user / --pass-file（0600）。
  token 由 robot_ui 启动时落盘 ~/.robot_ui_token（0600），本桥只读不写。
"""
import argparse
import json
import math
import os
import sys
import threading
import time
import urllib.parse
import urllib.request

# ---------------- 常量 ----------------
MAX_PAYLOAD = 4096            # 超长消息直接拒绝（卡-04 边界条件）
DEFAULT_LEASE_MS = 600        # 心跳租约：与 CMD_TIMEOUT 同口径
MAX_LEASE_MS = 5000
RENEW_HZ = 10.0               # 租约续发频率（≤10Hz，计划约定）
TELEMETRY_HZ = 1.0
STATUS_HZ = 1.0
CONNECT_RETRY_S = 5.0         # 断线重连目标窗口（卡-04：5s 内恢复订阅）
TASK_ACTIONS = ('stop', 'goal', 'intent', 'manual', 'heartbeat')


# ---------------- 纯逻辑：payload 校验 ----------------
def validate_task(obj):
    """校验下行 task JSON。返回 (规范化dict|None, 错误说明)。

    卡-04 边界条件全覆盖：非 JSON（调用方处理）、字段缺失、类型错、超长。
    """
    if not isinstance(obj, dict):
        return None, 'payload 必须是 JSON 对象'
    action = obj.get('action')
    if action not in TASK_ACTIONS:
        return None, 'action 无效：%r' % (action,)
    req_id = obj.get('req_id')
    if not isinstance(req_id, str) or not req_id or len(req_id) > 80:
        return None, 'req_id 缺失或非法（必填，≤80字符）'
    seq = obj.get('seq')
    if not isinstance(seq, int) or isinstance(seq, bool) or seq < 0:
        return None, 'seq 缺失或非法（必填，非负整数）'
    ts = obj.get('ts')
    if ts is not None and (not isinstance(ts, (int, float)) or isinstance(ts, bool)):
        return None, 'ts 非法（应为数字）'
    args = obj.get('args') or {}
    if not isinstance(args, dict):
        return None, 'args 必须是 JSON 对象'
    out = {'action': action, 'req_id': req_id, 'seq': seq,
           'ts': float(ts) if ts is not None else None,
           'dev': obj.get('dev'), 'args': dict(args),
           'on_disconnect': obj.get('on_disconnect') or 'continue'}
    if out['on_disconnect'] not in ('continue', 'stop'):
        return None, 'on_disconnect 无效（continue|stop）'
    if action == 'manual':
        try:
            vx, wz = float(args.get('vx')), float(args.get('wz'))
        except (TypeError, ValueError):
            return None, 'manual 的 vx/wz 必须是数字'
        if not (math.isfinite(vx) and math.isfinite(wz)):
            return None, 'vx/wz 必须是有限数值'
        out['args'] = dict(args, vx=max(-0.20, min(0.20, vx)),
                           wz=max(-1.0, min(1.0, wz)))
    elif action == 'goal':
        has_xy = all(k in args for k in ('x', 'y'))
        has_name = bool(args.get('name'))
        if not (has_xy or has_name):
            return None, 'goal 需要 args{x,y} 或 args{name}'
        if has_xy:
            try:
                x, y = float(args['x']), float(args['y'])
            except (TypeError, ValueError):
                return None, 'goal 的 x/y 必须是数字'
            if not (math.isfinite(x) and math.isfinite(y)):
                return None, 'x/y 必须是有限数值'
            out['args'] = dict(args, x=x, y=y)
    elif action == 'intent':
        if not isinstance(args.get('intent'), str) or not args.get('intent'):
            return None, 'intent 需要 args{intent}'
    return out, ''


# ---------------- 纯逻辑：seq 去旧（乱序防护） ----------------
class SeqGuard:
    """按来源维护最大 seq；只放行更“新”的消息，乱序/重放直接丢弃。

    卡-04 验收：「消息时间戳乱序时不影响指令正确性」——旧消息不执行即可。
    """

    def __init__(self):
        self._last = {}
        self.n_drop = 0

    def accept(self, key, seq):
        last = self._last.get(key)
        if last is not None and seq <= last:
            self.n_drop += 1
            return False
        self._last[key] = seq
        return True


# ---------------- 纯逻辑：手动租约状态机 ----------------
class ManualLease:
    """远程 manual 命令租约：心跳续发、到期补零、被本地接管则静默退让。

    tick() 返回：
      ('renew', vx, wz)  租约有效 → 续发当前速度
      ('zero', 0, 0)     租约刚到期 → 补一次零速（600ms 停车，卡-04）
      ('drop', None,None) 被本地接管/已收尾 → 什么都不做
      (None,  None, None) 无租约
    """

    def __init__(self, default_lease_ms=DEFAULT_LEASE_MS):
        self.default_lease_ms = default_lease_ms
        self.active = False
        self.vx = self.wz = 0.0
        self.req_id = ''
        self.expire_t = 0.0
        self._zero_sent = False

    def renew(self, vx, wz, req_id, lease_ms=None):
        try:
            ms = int(lease_ms) if lease_ms is not None else self.default_lease_ms
        except (TypeError, ValueError):
            ms = self.default_lease_ms
        ms = max(100, min(MAX_LEASE_MS, ms))
        self.active = True
        self.vx, self.wz = vx, wz
        self.req_id = req_id
        self.expire_t = time.monotonic() + ms / 1000.0
        self._zero_sent = False

    def heartbeat(self):
        """仅延租，不改速度（remote 端定频心跳）"""
        if self.active:
            self.expire_t = time.monotonic() + self.default_lease_ms / 1000.0

    def drop(self):
        self.active = False
        self._zero_sent = True

    def tick(self):
        if not self.active:
            return None, None, None
        now = time.monotonic()
        if now <= self.expire_t:
            return 'renew', self.vx, self.wz
        if not self._zero_sent:
            self._zero_sent = True
            self.active = False
            return 'zero', 0.0, 0.0
        return None, None, None


# ---------------- HTTP 回环（统一任务入口；同一 token） ----------------
class LocalUI:
    """robot_ui 本地 HTTP 客户端：token 从密钥文件读取（0600）"""

    def __init__(self, base='http://127.0.0.1:8080', token_file='~/.robot_ui_token'):
        self.base = base.rstrip('/')
        self.token = ''
        try:
            with open(os.path.expanduser(token_file), encoding='utf-8') as f:
                self.token = f.read().strip()
        except OSError:
            pass

    def get(self, path, params):
        q = dict(params)
        if self.token:
            q['k'] = self.token
        url = '%s%s?%s' % (self.base, path, urllib.parse.urlencode(q))
        try:
            with urllib.request.urlopen(url, timeout=3.0) as r:
                return json.loads(r.read().decode('utf-8'))
        except Exception as e:
            return {'ok': False, 'msg': '本地 HTTP 失败：%s' % e}

    def state(self):
        return self.get('/state', {})

    def dispatch(self, task):
        """task → /task/request 或 /voice/intent（同一鉴权与任务入口）"""
        a, args, rid = task['action'], task['args'], task['req_id']
        if a == 'stop':
            return self.get('/task/request',
                            {'source': 'mqtt', 'action': 'stop', 'request_id': rid})
        if a == 'goal':
            if args.get('name'):
                payload = {'intent': 'goto_place', 'args': {'name': args['name']},
                           'req_id': rid}
                return self.get('/voice/intent',
                                {'source': 'mqtt', 'req_id': rid,
                                 'intent': 'goto_place',
                                 'args': json.dumps(payload['args'],
                                                    ensure_ascii=False)})
            p = {'source': 'mqtt', 'action': 'goal', 'request_id': rid,
                 'x': args['x'], 'y': args['y']}
            if args.get('map_id'):
                p['map_id'] = args['map_id']
            return self.get('/task/request', p)
        if a == 'intent':
            return self.get('/voice/intent',
                            {'source': 'mqtt', 'req_id': rid,
                             'intent': args['intent'],
                             'args': json.dumps(args.get('args') or {},
                                                ensure_ascii=False)})
        if a == 'manual':
            return self.get('/task/request',
                            {'source': 'mqtt', 'action': 'manual', 'request_id': rid,
                             'vx': args['vx'], 'wz': args['wz']})
        return {'ok': False, 'msg': '未知 action：%s' % a}

    def zero_manual(self, base_rid, tick):
        return self.get('/task/request',
                        {'source': 'mqtt', 'action': 'manual', 'request_id': rid(base_rid, tick),
                         'vx': 0.0, 'wz': 0.0})


def rid(base, tick):
    """租约续发用的唯一 request_id（≤80 字符，满足幂等表唯一性）"""
    return ('%s~%d' % (base, tick))[:80]


# ---------------- 桥本体（rclpy 节点 + paho 网络线程） ----------------
def run_bridge(args):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        raise SystemExit('缺少 paho-mqtt：pip3 install --user paho-mqtt')
    try:
        import rclpy
        from geometry_msgs.msg import Twist          # noqa: F401（类型存在性）
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from sensor_msgs.msg import BatteryState
        from std_msgs.msg import String
    except ImportError as e:
        raise SystemExit('导入 ROS 2 模块失败（先 source 环境）：%s' % e)

    ui = LocalUI(args.ui, args.token_file)
    device = args.device
    prefix = '%s/%s' % (args.topic_prefix, device)
    t_telem, t_result, t_task, t_presence = (prefix + '/telemetry', prefix + '/result',
                                             prefix + '/task', prefix + '/presence')

    class Bridge(Node):
        def __init__(self):
            super().__init__('mqtt_bridge')
            self.lock = threading.Lock()
            self.connected = False
            self.n_in = self.n_out = self.n_task_ok = self.n_task_fail = 0
            self.last_msg = ''
            self.seq = 0
            self.seq_guard = SeqGuard()
            self.lease = ManualLease()
            self.goal_policy = None           # (req_id, on_disconnect)
            self.odom = None                  # (x, y, yaw, vx, wz)
            self.battery = None               # (voltage, pct)
            self.client = None
            self.renew_tick = 0

            self.create_subscription(Odometry, '/odom', self.on_odom, 10)
            self.create_subscription(BatteryState, '/battery', self.on_battery, 10)
            self.pub_status = self.create_publisher(String, '/mqtt/status', 10)
            self.create_timer(1.0 / STATUS_HZ, self.push_status)
            self.create_timer(1.0 / TELEMETRY_HZ, self.push_telemetry)
            self.create_timer(1.0 / RENEW_HZ, self.push_lease)

        # ---- ROS 输入 ----
        def on_odom(self, m):
            q = m.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, yaw,
                         m.twist.twist.linear.x, m.twist.twist.angular.z)

        def on_battery(self, m):
            pct = round(m.percentage * 100, 1) if math.isfinite(m.percentage) else None
            self.battery = (round(m.voltage, 3), pct)

        # ---- 状态上报（robot_ui 的 /mqtt/status 订阅此话题）----
        def push_status(self):
            with self.lock:
                d = {'connected': self.connected, 'broker': '%s:%s' % (args.host, args.port),
                     'n_in': self.n_in, 'n_out': self.n_out,
                     'last_msg': self.last_msg}
            msg = String()
            msg.data = json.dumps(d, ensure_ascii=False)
            self.pub_status.publish(msg)

        # ---- 遥测上行 ----
        def push_telemetry(self):
            if not self._mqtt_ok():
                return
            st = ui.state()
            o, b = self.odom, self.battery
            payload = {
                'dev': device, 'ts': round(time.time(), 3), 'seq': self._next_seq(),
                'pose': ({'x': round(o[0], 3), 'y': round(o[1], 3),
                          'yaw': round(o[2], 4)} if o else None),
                'vel': ({'vx': round(o[3], 3), 'wz': round(o[4], 3)} if o else None),
                'battery': ({'voltage': b[0], 'pct': b[1]} if b else None),
                'mode': st.get('mode'), 'goal': st.get('goal'),
                'map_id': st.get('map_id'),
            }
            self._publish(t_telem, payload)

        # ---- 租约续发（≤10Hz；断链/到期补零）----
        def push_lease(self):
            ui_poll = None
            with self.lock:
                cmd, vx, wz = self.lease.tick()
                base = self.lease.req_id or 'lease'
            if cmd is None:
                return
            if cmd == 'renew':
                # 只在仍持有远程控制权时续发；本地接管即静默退让（不抢、不补零）
                ui_poll = ui.state()
                holder = (ui_poll.get('mode') or {}).get('holder')
                if holder not in (None, '', 'mqtt'):
                    with self.lock:
                        self.lease.drop()
                    self.get_logger().info('远程租约退让：本地 %s 已接管' % holder)
                    return
                self.renew_tick += 1
                r = ui.get('/task/request',
                           {'source': 'mqtt', 'action': 'manual',
                            'request_id': rid(base, self.renew_tick),
                            'vx': vx, 'wz': wz})
                if not r.get('ok'):
                    self._log_task_fail(r.get('msg', ''))
            elif cmd == 'zero':
                r = ui.zero_manual(base, self.renew_tick + 1)
                self.renew_tick += 1
                self.get_logger().warn('心跳超时：远程租约已补零停车（600ms 口径）')
                self._publish(t_result, {'req_id': base, 'ok': True,
                                         'msg': '心跳超时，已停车'})

        # ---- MQTT 消息入口 ----
        def on_task_message(self, payload_bytes):
            if len(payload_bytes) > MAX_PAYLOAD:
                self._reject(None, '消息超长（>%d 字节）已拒绝' % MAX_PAYLOAD)
                return
            try:
                obj = json.loads(payload_bytes.decode('utf-8'))
            except (ValueError, UnicodeDecodeError):
                self._reject(None, '非法 JSON 已拒绝')
                return
            task, err = validate_task(obj)
            if task is None:
                self._reject(obj.get('req_id') if isinstance(obj, dict) else None, err)
                return
            with self.lock:
                self.n_in += 1
                self.last_msg = '%s#%s' % (task['action'], task['req_id'])
            # seq 去旧：乱序/重放不执行（不影响正确性）
            key = task.get('dev') or 'default'
            if not self.seq_guard.accept(key, task['seq']):
                self.get_logger().info('丢弃乱序/过期消息 seq=%d' % task['seq'])
                return
            a, args, ridv = task['action'], task['args'], task['req_id']

            if a == 'heartbeat':
                with self.lock:
                    self.lease.heartbeat()
                return
            if a == 'manual':
                with self.lock:
                    self.lease.renew(args['vx'], args['wz'], ridv,
                                     args.get('lease_ms'))
                r = ui.dispatch(task)
                self._report(ridv, r)
                return
            # stop / goal / intent → 统一任务入口
            r = ui.dispatch(task)
            self._report(ridv, r)
            if a == 'goal' and r.get('ok'):
                with self.lock:
                    self.goal_policy = (ridv, task['on_disconnect'])
            if a == 'stop':
                with self.lock:
                    self.lease.drop()
                    self.goal_policy = None

        def _mqtt_ok(self):
            return self.client is not None and self.connected

        def _next_seq(self):
            with self.lock:
                self.seq += 1
                return self.seq

        def _publish(self, topic, payload):
            payload = dict(payload, dev=device, ts=round(time.time(), 3),
                           seq=payload.get('seq') or self._next_seq())
            try:
                self.client.publish(topic, json.dumps(payload, ensure_ascii=False),
                                    qos=args.qos)
                with self.lock:
                    self.n_out += 1
            except Exception as e:
                self.get_logger().error('MQTT 发布失败：%s' % e)

        def _report(self, req_id, result):
            ok = bool(result.get('ok'))
            with self.lock:
                self.n_task_ok += int(ok)
                self.n_task_fail += int(not ok)
            payload = {'req_id': req_id, 'ok': ok,
                       'msg': str(result.get('msg', '')
                                  or ('重复请求已忽略' if result.get('duplicate') else ''))}
            if result.get('duplicate'):
                payload['duplicate'] = True      # 幂等标志要透传，云端才能看见
            self._publish(t_result, payload)

        def _reject(self, req_id, err):
            self.get_logger().warn('task 拒绝：%s' % err)
            self._publish(t_result, {'req_id': req_id or '-', 'ok': False, 'msg': err})

        def _log_task_fail(self, msg):
            self.get_logger().warn('manual 续发被拒：%s' % msg)

        # ---- 连接生命周期 ----
        def on_connect(self, client, userdata, flags, reason_code, properties=None):
            # paho 2.x 的 reason_code 是 ReasonCode 对象，int() 会 TypeError；用 is_failure
            ok = not getattr(reason_code, 'is_failure', False)
            with self.lock:
                self.connected = ok
            if not ok:
                self.get_logger().error('MQTT 连接被拒 reason=%s（查账号/ACL）' % reason_code)
                return
            # 红队坑对策：重连后必须重订阅 + 重发在线状态
            client.subscribe(t_task, qos=args.qos)
            client.publish(t_presence, 'online', qos=1, retain=True)
            self.get_logger().info('MQTT 已连接 %s:%s ｜ 订阅 %s'
                                   % (args.host, args.port, t_task))

        def on_disconnect(self, client, userdata, disconnect_flags, reason_code,
                          properties=None):
            with self.lock:
                self.connected = False
                policy = self.goal_policy
                lease_active = self.lease.active
                self.lease.drop()
            self.get_logger().warn('MQTT 断链 reason=%s，%.0fs 内自动重试'
                                   % (reason_code, CONNECT_RETRY_S))
            if lease_active:
                # 远程租约内的手动命令 → 补零（只作用于远程租约，本地任务不碰）
                ui.zero_manual('disc', self.renew_tick + 1)
                self.renew_tick += 1
                self._publish_local_result('断链，远程手动已停车')
            if policy and policy[1] == 'stop':
                ui.dispatch({'action': 'stop', 'req_id': 'disc-' + policy[0][:60],
                             'args': {}})
                self._publish_local_result('断链，按 on_disconnect=stop 终止远程任务')

        def _publish_local_result(self, msg):
            self.get_logger().info(msg)
            # 注意：断链期间 publish 会失败，仅记日志即可（本地 HTTP 补零已生效）

    # ---- paho 组装 ----
    rclpy.init()                      # ⚠ 缺这句 rclpy.Node 直接 NotInitializedException
    bridge = Bridge()
    client_id = 'robot-%s-%d' % (device, os.getpid() % 10000)
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id,
                         protocol=mqtt.MQTTv311, clean_session=True)
    if args.user:
        pw = os.environ.get('MQTT_PASS', '')
        if args.pass_file:
            try:
                with open(os.path.expanduser(args.pass_file), encoding='utf-8') as f:
                    pw = f.read().strip()
            except OSError as e:
                raise SystemExit('密码文件读取失败：%s' % e)
        client.username_pw_set(args.user, pw or None)
    if args.no_tls:
        print('⚠⚠ 明文连接（--no-tls）：仅限本地调试，上云必须 TLS（卡-04 禁止明文）')
    else:
        import ssl
        client.tls_set(ca_certs=args.tls_ca or None,
                       certfile=args.tls_cert or None,
                       keyfile=args.tls_key or None,
                       tls_version=ssl.PROTOCOL_TLS_CLIENT)
        client.tls_insecure_set(False)
    # 遗嘱：异常掉线由 broker 代发 offline（retain）
    client.will_set(t_presence, 'offline', qos=1, retain=True)
    client.reconnect_delay_set(min_delay=1, max_delay=int(CONNECT_RETRY_S))
    client.on_connect = bridge.on_connect
    client.on_disconnect = bridge.on_disconnect
    client.on_message = lambda c, u, m: bridge.on_task_message(m.payload)
    bridge.client = client

    try:
        client.connect_async(args.host, args.port, keepalive=args.keepalive)
        client.loop_start()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print('\n收到 Ctrl-C，退出中 ...')
    finally:
        try:
            client.publish(t_presence, 'offline', qos=1, retain=True)
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ---------------- 纯逻辑自测（--selftest） ----------------
def selftest():
    # 1. 校验
    ok, err = validate_task({'action': 'manual', 'req_id': 'r1', 'seq': 1,
                             'args': {'vx': 0.5, 'wz': 0}})
    assert ok and ok['args']['vx'] == 0.20, (ok, err)          # 超限钳位
    for bad in ({}, {'action': 'fly', 'req_id': 'a', 'seq': 1},
                {'action': 'stop', 'seq': 1},                  # 缺 req_id
                {'action': 'stop', 'req_id': 'a'},             # 缺 seq
                {'action': 'manual', 'req_id': 'a', 'seq': 2,
                 'args': {'vx': float('nan'), 'wz': 0}},
                {'action': 'goal', 'req_id': 'a', 'seq': 3},   # 缺 x,y/name
                {'action': 'goal', 'req_id': 'a', 'seq': 4,
                 'on_disconnect': 'maybe'}):
        got, e = validate_task(bad)
        assert got is None and e, (bad, got)
    print('1. validate_task OK')

    # 2. seq 去旧
    g = SeqGuard()
    assert g.accept('d', 5) and g.accept('d', 7)
    assert not g.accept('d', 7) and not g.accept('d', 3)       # 重复/乱序丢弃
    assert g.accept('e', 1)                                    # 另一来源独立
    assert g.n_drop == 2
    print('2. SeqGuard OK')

    # 3. 租约：续发 → 到期补零 → 收尾
    import time as _t
    lease = ManualLease(default_lease_ms=200)
    lease.renew(0.1, 0.0, 'r1')
    assert lease.tick()[0] == 'renew'
    _t.sleep(0.25)
    cmd, vx, wz = lease.tick()
    assert cmd == 'zero' and vx == 0.0 and wz == 0.0, (cmd, vx, wz)
    assert lease.tick()[0] is None                             # 补零只发一次
    # heartbeat 只延租
    lease.renew(0.1, 0.0, 'r2')
    _t.sleep(0.12)
    lease.heartbeat()
    _t.sleep(0.12)
    assert lease.tick()[0] == 'renew'
    lease.drop()
    assert lease.tick()[0] is None
    print('3. ManualLease OK')

    # 4. rid 唯一且合规
    ids = {rid('r1', i) for i in range(1000)}
    assert len(ids) == 1000 and all(len(x) <= 80 for x in ids)
    print('4. rid OK')
    print('ALL PASS')


def main():
    ap = argparse.ArgumentParser(description='MQTT 遥测/任务桥（卡-04）')
    ap.add_argument('--host', default=os.environ.get('MQTT_HOST', ''),
                    help='Broker 地址（或环境变量 MQTT_HOST）')
    ap.add_argument('--port', type=int, default=int(os.environ.get('MQTT_PORT', '8883')),
                    help='端口（云端 TLS 默认 8883）')
    ap.add_argument('--device', default='car01', help='设备 ID（ACL 按此隔离）')
    ap.add_argument('--topic-prefix', default='robot', help='话题前缀（默认 robot）')
    ap.add_argument('--user', default=os.environ.get('MQTT_USER', ''),
                    help='账号（或环境变量 MQTT_USER）')
    ap.add_argument('--pass-file', default='', help='密码文件（0600；或环境变量 MQTT_PASS）')
    ap.add_argument('--tls-ca', default='', help='CA 证书路径')
    ap.add_argument('--tls-cert', default='', help='客户端证书（双向 TLS）')
    ap.add_argument('--tls-key', default='', help='客户端私钥')
    ap.add_argument('--no-tls', action='store_true', help='明文（仅本地调试）')
    ap.add_argument('--qos', type=int, default=1, choices=(0, 1, 2), help='MQTT QoS')
    ap.add_argument('--keepalive', type=int, default=30, help='MQTT keepalive 秒')
    ap.add_argument('--ui', default='http://127.0.0.1:8080',
                    help='robot_ui 本地地址（统一任务入口）')
    ap.add_argument('--token-file', default='~/.robot_ui_token',
                    help='robot_ui token 密钥文件（app.py 启动时落盘）')
    ap.add_argument('--selftest', action='store_true', help='纯逻辑自测（无 broker）')
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if not args.host:
        raise SystemExit('必须给 --host（或 MQTT_HOST）；本地自测请用 --selftest')
    if not args.no_tls and not args.tls_ca:
        print('⚠ 未给 --tls-ca：将用系统 CA 校验（自签证书请提供 CA 文件）')
    run_bridge(args)


if __name__ == '__main__':
    main()
