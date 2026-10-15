"""导航/航点路由（从 app.py Handler 分支逐字平移，函数签名统一 fn(handler, q)）"""
import json


def _claim_nav(h):
    """动导航前认领 NAV：可抢 TELEOP/IDLE，PATROL 占用时拒绝（判断收在锁内，无 TOCTOU）"""
    ok, mmsg = h.node.modes.claim('web-nav', 'NAV', force=True, force_unless='PATROL')
    if not ok:
        h._send(409, 'application/json; charset=utf-8',
                json.dumps({'ok': False, 'msg': '✗ ' + mmsg}, ensure_ascii=False).encode('utf-8'))
    return ok


def goal(h, q):
    # 点选目标：u,v = 画面像素（也支持直接给世界坐标 x,y，便于命令行自测）
    if not _claim_nav(h):
        return
    try:
        if 'x' in q and 'y' in q:
            info = h.node.set_goal(x=float(q['x'][0]), y=float(q['y'][0]))
        else:
            info = h.node.set_goal(u=float(q.get('u', ['-1'])[0]),
                                   v=float(q.get('v', ['-1'])[0]))
    except (TypeError, ValueError):
        info = {'ok': False, 'msg': '参数错误（需要 u,v 或 x,y）'}
    h._send_json(info)


def goal_home(h, q):
    # 一键回原点：目标 = 地图原点 (0,0) = 建图起点
    if not _claim_nav(h):
        return
    h._send_json(h.node.goto_home())


def goal_clear(h, q):
    # 安全动作不设防；先清目标再释放 NAV（避免释放后、清除前的间隙被别的屏认领）
    info = h.node.clear_goal()
    h.node.modes.release('web-nav')
    h._send_json(info)


def wp_add(h, q):
    # 多点导航：把一个点加入队列（u,v 画面像素 或 x,y 世界坐标）
    if not _claim_nav(h):
        return
    try:
        if 'x' in q and 'y' in q:
            info = h.node.wp_add(x=float(q['x'][0]), y=float(q['y'][0]))
        else:
            info = h.node.wp_add(u=float(q.get('u', ['-1'])[0]),
                                 v=float(q.get('v', ['-1'])[0]))
    except (TypeError, ValueError):
        info = {'ok': False, 'msg': '参数错误（需要 u,v 或 x,y）'}
    h._send_json(info)


def wp_go(h, q):
    if not _claim_nav(h):
        return
    h._send_json(h.node.wp_go())


def wp_clear(h, q):
    # 安全动作不设防；先清队列/取消执行再释放 NAV（理由同 goal_clear）
    info = h.node.wp_clear()
    h.node.modes.release('web-nav')
    h._send_json(info)
