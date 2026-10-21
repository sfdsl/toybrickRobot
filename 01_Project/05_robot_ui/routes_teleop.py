"""遥控/视图/存图/状态路由（从 app.py Handler 分支逐字平移，函数签名统一 fn(handler, q)）"""
import json

from robot_ui import state_store


def cmd(h, q):
    try:
        vx = float(q.get('vx', ['0'])[0])
        wz = float(q.get('wz', ['0'])[0])
    except (TypeError, ValueError):
        h._send(400, 'text/plain', b'bad args')
        return
    # ModeMgr 互斥：动 /cmd_vel 前先认领。摇杆可从 IDLE/TELEOP/NAV 认领
    # （对 NAV 自动抢占——网页点选前会先 halt，用户摸摇杆就是明确要接管），仅 PATROL 拒绝。
    ok, mmsg = h.node.modes.claim('web-teleop', 'TELEOP', force=True, force_unless='PATROL')
    if not ok:
        h._send(409, 'application/json; charset=utf-8',
                json.dumps({'ok': False, 'msg': '✗ ' + mmsg}, ensure_ascii=False).encode('utf-8'))
        return
    if vx != 0.0 or wz != 0.0:
        h.node.patrol.cancel()
        # A manual takeover cancels the old navigation task once; a fresh
        # nonzero manual command below immediately claims the mux.
        if h.node.goal is not None or h.node.wp_running:
            h.node.wp_clear()
            h.node.clear_goal()
    vx, wz = h.node.set_cmd(vx, wz)
    h._send(200, 'text/plain', ('%+.2f %+.2f' % (vx, wz)).encode())


def stop(h, q):
    # 显式停止锁存；下一条新导航任务或人工指令才会重新获得控制权。
    h.node.explore.cancel()
    h.node.patrol.cancel()
    h.node.wp_clear()
    h.node.clear_goal()
    h._send(200, 'text/plain', b'stopped')


def view(h, q):
    # 缩放/平移：range=米 &&/|| dx= dy=（米，相对车心）；reset=1 复位
    try:
        info = h.node.set_view(
            range_m=float(q['range'][0]) if 'range' in q else None,
            dx=float(q['dx'][0]) if 'dx' in q else None,
            dy=float(q['dy'][0]) if 'dy' in q else None,
            reset=('reset' in q))
    except (TypeError, ValueError):
        info = {'ok': False, 'msg': '参数错误（需要 range / dx / dy / reset）'}
    h._send_json(info)


def savemap(h, q):
    # 保存地图（会阻塞几秒：等 serialize_map 服务写完文件）
    try:
        info = h.node.save_map()
    except Exception as e:
        info = {'ok': False, 'msg': '保存异常：%s' % e}
    h._send_json(info)


def state(h, q):
    # curl 自测用（前端经 /events SSE 取状态；/state 免鉴权便于脚本巡检）
    h._send_json(h.node.state())


def features(h, q):
    # 功能注册表（hub 屏4）：免鉴权只读，app.py main() 启动时预置 15 项
    h._send_json({'ok': True, 'features': state_store.FEATURES})
