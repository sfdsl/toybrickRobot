#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""serial_cmd.py — ESP32 串口意图接口（P3 / 卡-05 RK3588 侧）

═══════════════════════════ 串口协议规范 v1 ═══════════════════════════

【物理层】
  3.3V TTL UART，**115200 8N1**，无流控；TX/RX 交叉、共地；板间直连 < 1 m。
  RK3588 侧设备节点可配：--serial /dev/ttyUSB0 或 SoC UART /dev/ttyS3|/dev/ttyS7
  （后者需确认 pinmux）。**禁用 /dev/ttyACM0**（底盘 base_driver 占用）。

【帧格式】NDJSON——UTF-8，一行一条 JSON，`\\n` 结尾，单条 ≤ 512 字节。
  非 JSON / 超长 / 字段缺失 → 回 ok:false 应答并丢弃该行，不崩溃、不停车。

【命令（ESP32 → RK3588）】
  {"intent":"goto_place","args":{"name":"充电位"},"req_id":"esp32-000123","ts":1729400000.5}

  intent   必填，词表枚举：stop / forward / back / left / right /
           home / goto_place / patrol_start / patrol_stop（词表外拒绝）
  args     可选：forward/back 的 duration（秒，≤3）；left/right 的 angle（度）或
           duration；goto_place 的 name（地点名，必填）
  req_id   必填（stop 豁免），≤80 字符唯一 ID；重复 req_id 不重复执行（300s 窗口），
           同 ID 异参数拒绝
  ts       可选，发送时刻（epoch 秒），乱序去旧用；缺省不校验

【应答（RK3588 → ESP32，供 ESP32 TTS 播报）】
  {"ack":"esp32-000123","ok":true,"msg":"目标已设：x=6.61 y=-1.02"}
  {"ack":"esp32-000123","ok":false,"msg":"地点不存在：会议室"}

【语义规则】
  · stop 最高优先、免鉴权、幂等豁免：重复 stop 重复停，直通本地停止链
    （patrol.cancel + wp_clear + clear_goal + publish_control('stop')），
    全程不依赖网络/MQTT/云端 —— 断网时本地「停止」有效的物理保证
  · 手动类（forward/back/left/right）限速 ≤0.10 m/s、限时默认 1.5s 自动归零；
    另有 PhoneTeleop.CMD_TIMEOUT=0.6s 看门狗兜底
  · goto_place 名称只查 04_map/robot_map.places.json（PlaceStore）：
    命中 → map_id 校验 → set_goal()（含预检）；未命中/换图 → ok:false，不发目标
  · 串口断开自动退避重连；断开期间不影响已批准任务（与远程租约同语义）

═══════════════════════════════════════════════════════════════════════

运行：
  机器人进程内（app.py --serial /dev/ttyUSB0 自动拉起本模块线程）——生产路径
  python3 serial_cmd.py --serial /dev/ttyUSB0        # 独立进程直连（需 ROS 环境）
  python3 serial_cmd.py --file lines.txt             # 回放自测（无硬件）
  python3 serial_cmd.py --demo                       # 内置样例自测（无硬件）
"""
import json
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

MAX_LINE = 512                # 单条帧上限（字节，UTF-8 计）
BAUD = 115200
RECONNECT_BACKOFF_S = 2.0     # 串口断开重连退避

# 串口状态（routes_voice._serial_status() 消费；语音屏三路灯的串口一路）
STATUS = {'enabled': False, 'port': '', 'n_rx': 0, 'n_ok': 0, 'n_reject': 0,
          'last': '', 'ts': 0.0}
_STATUS_LOCK = threading.Lock()


def _status_update(**kw):
    with _STATUS_LOCK:
        STATUS.update(kw)
        STATUS['ts'] = time.time()


def status():
    with _STATUS_LOCK:
        return dict(STATUS)


# ---------------- 单行处理（纯逻辑，自测桩与真串口共用） ----------------
def process_line(engine, line):
    """处理一条 NDJSON 命令行 → 应答 dict（调用方序列化后回写）

    engine：IntentEngine（进程内 robot_ui.voice）或任何有 .execute(payload,
            source=, req_id=) 的对象（自测桩用 FakeEngine）。
    永不抛异常：解析失败/词表外/执行异常都变成 ok:false 应答。
    """
    line = (line or '').strip()
    if not line:
        return None                                    # 空行静默跳过
    if len(line.encode('utf-8', 'ignore')) > MAX_LINE:
        return {'ack': '', 'ok': False,
                'msg': '超长帧（>%d 字节）已拒绝' % MAX_LINE}
    try:
        obj = json.loads(line)
    except ValueError:
        return {'ack': '', 'ok': False, 'msg': '非法 JSON 已拒绝'}
    if not isinstance(obj, dict):
        return {'ack': '', 'ok': False, 'msg': '帧必须是 JSON 对象'}
    intent = obj.get('intent')
    if not isinstance(intent, str) or not intent:
        return {'ack': str(obj.get('req_id') or ''), 'ok': False,
                'msg': 'intent 缺失'}
    req_id = obj.get('req_id')
    is_stop = (intent == 'stop')
    if not is_stop:
        # 非 stop：req_id 必填（协议要求；幂等与去旧的键）
        if not isinstance(req_id, str) or not req_id or len(req_id) > 80:
            return {'ack': '', 'ok': False,
                    'msg': 'req_id 缺失或非法（必填，≤80 字符）'}
    args = obj.get('args')
    if args is not None and not isinstance(args, dict):
        return {'ack': str(req_id or ''), 'ok': False, 'msg': 'args 必须是对象'}
    payload = {'intent': intent, 'args': args or {},
               'text': '', 'req_id': req_id} if req_id else \
              {'intent': intent, 'args': args or {}, 'text': ''}
    try:
        res = engine.execute(payload, source='serial',
                             req_id=req_id if req_id else None)
    except Exception as e:                             # 防御：执行器内部错误不外溢
        return {'ack': str(req_id or ''), 'ok': False, 'msg': '执行异常：%s' % e}
    if is_stop and isinstance(res, dict):
        res = dict(res)                                # stop 免 req_id，ack 照回
    return {'ack': str(req_id or 'stop'), 'ok': bool(res.get('ok')),
            'msg': str(res.get('msg', ''))}


# ---------------- 串口监听线程（机器人进程内 / 独立进程共用） ----------------
class SerialBridge(threading.Thread):
    """读串口行 → process_line → 应答行回写；断开自动退避重连"""

    def __init__(self, engine, port, baud=BAUD, write=None):
        super().__init__(daemon=True)
        self.engine = engine
        self.port = port
        self.baud = baud
        self.write = write                # 覆盖回写（自测桩注入收集器）
        self._stop_evt = threading.Event()

    def stop(self):
        self._stop_evt.set()

    def _open(self):
        import serial                   # pyserial（板上已装 3.5）
        return serial.Serial(self.port, self.baud, timeout=0.5,
                             write_timeout=1.0)

    def _handle(self, line, ser):
        ack = process_line(self.engine, line)
        if ack is None:
            return
        ok = bool(ack.get('ok'))
        _status_update(enabled=True, port=self.port,
                       n_rx=STATUS['n_rx'] + 1,
                       n_ok=STATUS['n_ok'] + int(ok),
                       n_reject=STATUS['n_reject'] + int(not ok),
                       last='%s → %s' % (line[:40], ack.get('msg', '')[:40]))
        out = (json.dumps(ack, ensure_ascii=False) + '\n').encode('utf-8')
        try:
            if self.write is not None:
                self.write(ack)
            elif ser is not None:
                ser.write(out)
        except Exception:                # 回写失败不影响命令已执行
            pass

    def run(self):
        while not self._stop_evt.is_set():
            try:
                ser = self._open()
                _status_update(enabled=True, port=self.port)
                buf = b''
                try:
                    while not self._stop_evt.is_set():
                        chunk = ser.read(256)
                        if not chunk:
                            continue
                        buf += chunk
                        while b'\n' in buf:
                            line, buf = buf.split(b'\n', 1)
                            self._handle(line.decode('utf-8', 'ignore'), ser)
                finally:
                    try:
                        ser.close()
                    except Exception:
                        pass
            except Exception as e:
                _status_update(enabled=False, port=self.port,
                               last='串口断开：%s' % e)
                self._stop_evt.wait(RECONNECT_BACKOFF_S)   # 退避后重连
        _status_update(enabled=False)


# ---------------- 自测桩（--demo / --file：无硬件可测全路径） ----------------
class FakeEngine:
    """IntentEngine 替身：记录调用、可编程结果；不依赖 ROS/rclpy"""

    def __init__(self):
        self.calls = []
        self.script = {}                 # intent → 结果 dict

    def execute(self, payload, source='serial', req_id=None):
        self.calls.append((payload, source, req_id))
        intent = payload.get('intent')
        if intent in self.script:
            return dict(self.script[intent])
        return {'ok': True, 'msg': '执行 %s' % intent, 'intent': intent}


def run_lines(engine, lines, collect):
    for ln in lines:
        ack = process_line(engine, ln)
        if ack is not None:
            collect.append(ack)
            print('  <', json.dumps(ack, ensure_ascii=False))


def selftest():
    print('== serial_cmd 协议自测桩 ==')
    eng = FakeEngine()
    got = []

    print('1. 正常命令与 ACK')
    run_lines(eng, [
        '{"intent":"goto_place","args":{"name":"充电位"},"req_id":"esp32-1"}',
        '{"intent":"forward","args":{"duration":1.5},"req_id":"esp32-2"}',
    ], got)
    assert got[0]['ack'] == 'esp32-1' and got[0]['ok']
    assert got[1]['ack'] == 'esp32-2' and got[1]['ok']

    print('2. stop 免 req_id、最高优先')
    n = len(eng.calls)
    run_lines(eng, ['{"intent":"stop"}', '{"intent":"stop","req_id":"s1"}'], got)
    assert got[-2]['ack'] == 'stop' and got[-2]['ok'] and got[-1]['ok']
    assert len(eng.calls) == n + 2                    # stop 必须被执行（幂等豁免）

    print('3. 非法输入拒绝且不崩溃')
    for bad in ('not json', '[]', '{"req_id":"x"}', '{"intent":""}',
                '{"intent":"forward","req_id":""}',
                '{"intent":"forward","args":5,"req_id":"x"}',
                'x' * 600):
        ack = process_line(eng, bad)
        assert ack is not None and ack['ok'] is False, (bad, ack)

    print('4. 空行静默')
    assert process_line(eng, '') is None
    assert process_line(eng, '   ') is None

    print('5. 执行异常不外溢')
    class Boom:
        def execute(self, *a, **k):
            raise RuntimeError('boom')
    ack = process_line(Boom(), '{"intent":"stop"}')
    assert ack['ok'] is False and '执行异常' in ack['msg']

    print('6. 回放文件模式（--file 同路径）')
    got.clear()
    run_lines(eng, ['{"intent":"home","req_id":"esp32-9"}'], got)
    assert got[0]['ok'] and got[0]['ack'] == 'esp32-9'

    print('ALL PASS')


def main():
    import argparse
    ap = argparse.ArgumentParser(description='ESP32 串口意图接口（协议见文件头）')
    ap.add_argument('--serial', default='', help='串口设备（如 /dev/ttyUSB0；禁用 /dev/ttyACM0）')
    ap.add_argument('--baud', type=int, default=BAUD)
    ap.add_argument('--file', default='', help='回放 NDJSON 文件（无硬件自测）')
    ap.add_argument('--demo', action='store_true', help='内置样例自测（无硬件）')
    ap.add_argument('--ui', default='', help='独立进程模式：robot_ui 地址（经 /voice/intent）')
    args = ap.parse_args()

    if args.demo:
        return selftest()
    if args.file:
        eng = FakeEngine()
        with open(args.file, encoding='utf-8') as f:
            run_lines(eng, f.read().splitlines(), [])
        return 0
    if not args.serial:
        raise SystemExit('需要 --serial / --file / --demo 之一')
    if '/dev/ttyACM0' in args.serial:
        raise SystemExit('拒绝：/dev/ttyACM0 被底盘占用，换 /dev/ttyUSB0 或 SoC UART')

    if args.ui:
        # 独立进程模式：经 localhost HTTP 转发（与 mqtt_bridge 同一鉴权口径）
        import mqtt_bridge as mb
        eng = mb.LocalUI(args.ui)
        class _HttpEngine:
            def execute(self, payload, source='serial', req_id=None):
                return eng.get('/voice/intent',
                               {'source': source,
                                'req_id': req_id or '',
                                'intent': payload.get('intent'),
                                'args': json.dumps(payload.get('args') or {},
                                                   ensure_ascii=False)})
        engine = _HttpEngine()
    else:
        raise SystemExit('进程内模式由 app.py --serial 自动拉起；独立跑请加 --ui http://127.0.0.1:8080')

    br = SerialBridge(engine, args.serial, args.baud)
    br.start()
    print('串口监听已启动：%s @%d（Ctrl-C 退出）' % (args.serial, args.baud))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        br.stop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main() or 0)
