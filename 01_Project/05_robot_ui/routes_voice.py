"""语音/意图路由（P3）：/voice/intent（文本或结构化意图）+ /voice/status

三源一口：浏览器语音、MQTT 桥（localhost 回环）、串口监听共用 IntentEngine。
本路由只是 HTTP 皮：词表校验、req_id 幂等、去抖、安全限速全在 robot_ui.intent，
执行走与 /task/request 相同的 node 层语义（ModeMgr / PlaceStore / set_goal）。
"""
import json


def _serial_status():
    """串口接口状态（serial_cmd 未启动/未实现时安全降级）"""
    try:
        from robot_ui import serial_cmd
        return dict(serial_cmd.STATUS)
    except Exception:
        return {'enabled': False}


SOURCES = ('web', 'voice', 'mqtt', 'serial', 'test')


def intent(h, q):
    """GET /voice/intent

    文本形式：  ?text=去充电位&source=voice&req_id=可选
    结构化形式：?intent=goto_place&args={"name":"充电位"}&source=mqtt&req_id=必填
    """
    source = q.get('source', [''])[0] or 'voice'
    if source not in SOURCES:
        return h._send_json({'ok': False, 'msg': 'source 无效：%s' % source})
    req_id = q.get('req_id', [''])[0] or None
    raw_text = q.get('text', [''])[0]
    raw_intent = q.get('intent', [''])[0]
    args_raw = q.get('args', [''])[0]
    if raw_text:
        payload = raw_text
    elif raw_intent:
        try:
            args = json.loads(args_raw) if args_raw else {}
        except ValueError:
            return h._send_json({'ok': False, 'msg': 'args 不是合法 JSON'})
        if not isinstance(args, dict):
            return h._send_json({'ok': False, 'msg': 'args 必须是 JSON 对象'})
        payload = {'intent': raw_intent, 'args': args}
    else:
        return h._send_json({'ok': False, 'msg': '缺少 text 或 intent 参数'})
    res = h.node.voice.execute(payload, source=source, req_id=req_id)
    h._send_json(res)


def status(h, q):
    """GET /voice/status：意图层 + 串口 + MQTT 桥状态（语音屏顶部指示灯数据源）"""
    st = h.node.voice.status()
    st['serial'] = _serial_status()
    st['mqtt'] = dict(getattr(h.node, '_mqtt', {}))
    st['mode'] = h.node.modes.snapshot()
    h._send_json({'ok': True, **st})
