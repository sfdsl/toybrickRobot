"""Single ingress for future MQTT/voice adapters and external control clients.

This uses the same token, ModeMgr, goal precheck, and control latch as web UI.
Adapters send an idempotent request_id; HTTP GET is accepted for existing server.
"""
import math
import threading
import time


_lock = threading.Lock()
_request_lock = threading.Lock()
_seen = {}


def _value(q, name):
    return q.get(name, [''])[0]


def request(h, q):
    with _request_lock:
        return _request_inner(h, q)


def _request_inner(h, q):
    source = _value(q, 'source')
    action = _value(q, 'action')
    request_id = _value(q, 'request_id')
    if source not in ('web', 'mqtt', 'voice', 'test') or action not in ('stop', 'goal', 'manual'):
        return h._send_json({'ok': False, 'msg': 'source/action 无效'})
    if not request_id or len(request_id) > 80:
        return h._send_json({'ok': False, 'msg': 'request_id 必填，最长 80 字符'})
    key = (source, request_id)
    signature = tuple(sorted((name, tuple(values)) for name, values in q.items()
                             if name != 'k'))
    with _lock:
        now = time.monotonic()
        for k, (t, _) in list(_seen.items()):
            if now - t > 300:
                del _seen[k]
        if key in _seen:
            return h._send_json({'ok': True, 'duplicate': True} if _seen[key][1] == signature
                                else {'ok': False, 'msg': 'request_id 已用于不同请求'})
    node = h.node
    try:
        if action == 'stop':
            node.patrol.cancel()
            node.wp_clear()
            node.clear_goal()
            result = {'ok': True, 'msg': '已停车并清除导航目标'}
        elif action == 'manual':
            vx, wz = float(_value(q, 'vx')), float(_value(q, 'wz'))
            if not (math.isfinite(vx) and math.isfinite(wz)):
                raise ValueError('速度必须是有限数值')
            ok, msg = node.modes.claim(source, 'TELEOP', force=True, force_unless='PATROL')
            if not ok:
                raise ValueError(msg)
            if (vx != 0.0 or wz != 0.0) and (node.goal is not None or node.wp_running):
                node.wp_clear()
                node.clear_goal()
            node.set_cmd(vx, wz)
            result = {'ok': True, 'vx': vx, 'wz': wz}
        else:
            x, y = float(_value(q, 'x')), float(_value(q, 'y'))
            if not (math.isfinite(x) and math.isfinite(y)):
                raise ValueError('坐标必须是有限数值')
            expected = _value(q, 'map_id')
            if expected and expected != node.places.map_id():
                raise ValueError('地图版本不匹配')
            ok, msg = node.modes.claim(source, 'NAV', force=(source == 'web'),
                                       force_unless='PATROL')
            if not ok:
                raise ValueError(msg)
            result = node.set_goal(x=x, y=y)
        if result.get('ok'):
            with _lock:
                _seen[key] = (time.monotonic(), signature)
        h._send_json(result)
    except (ValueError, OSError) as e:
        h._send_json({'ok': False, 'msg': str(e)})
