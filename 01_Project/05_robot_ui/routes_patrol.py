"""巡逻路由（屏2/屏3 后端）：/patrol/load|points|go|pause|resume|skip|cancel

与旧 /wp/*（FollowWaypoints 连发）并存；本组走 PatrolRunner（方案A 顺序逐点）。
"""
import json


def load(h, q):
    """装载巡逻点：pts=JSON 数组 [[x,y,dwell,name],...]（URL 编码），loop=1 循环"""
    try:
        raw = json.loads(q.get('pts', ['[]'])[0])
        loop = q.get('loop', ['0'])[0] in ('1', 'true')
        pts = [{'x': float(p[0]), 'y': float(p[1]),
                'dwell': float(p[2]) if len(p) > 2 else 3.0,
                'name': str(p[3]) if len(p) > 3 else '点%d' % (i + 1)}
               for i, p in enumerate(raw)]
    except (TypeError, ValueError, IndexError, json.JSONDecodeError):
        h._send_json({'ok': False, 'msg': '参数错误（pts 应为 [[x,y,dwell,name],...] JSON）'})
        return
    h._send_json(h.node.patrol.load(pts, loop))


def points(h, q):
    # 当前装载的巡逻点（网页回显）
    h._send_json(h.node.patrol.points())


def go(h, q):
    # 出发（内部 claim PATROL，被占用即拒绝）
    h._send_json(h.node.patrol.go())


def pause(h, q):
    h._send_json(h.node.patrol.pause())


def resume(h, q):
    h._send_json(h.node.patrol.resume())


def skip(h, q):
    h._send_json(h.node.patrol.skip())


def cancel(h, q):
    h._send_json(h.node.patrol.cancel())
