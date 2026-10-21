"""地图编辑路由（P2-A4）：擦除假障碍 / 画虚拟墙，另存新版本

薄壳：参数解析 → 转发给 `h.node.map_editor`（核心 `MapEditor` 在
`01_Project/03_SLAM/map_editor.py`，由 `node_teleop.py` 挂在节点上）。

⚠️ 纯编辑，**不发 `/cmd_vel`**、不参与运动控制 → 不认领 ModeMgr
   （同 places 的只读/落盘动作；安全动作不设防）。
⚠️ 导出**永不覆盖 `robot_map.*`**（另存不覆盖原则，见 17 号文档 P2）。
"""
import os

import numpy as np

try:
    import cv2
except ImportError as e:                                   # pragma: no cover
    raise SystemExit('导入 cv2 失败：%s' % e)

# 编辑版默认落盘前缀（与 03_SLAM/map_editor.py 的 DEFAULT_EXPORT_PREFIX 一致）
DEFAULT_EXPORT = os.path.expanduser('~/RobotCode/04_map/robot_map_clean')
# 绝不允许被覆盖的前缀（原始地图）
PROTECTED = ('robot_map',)


def _num(q, key, default=None):
    v = q.get(key, [''])[0]
    if v == '':
        return default
    try:
        return float(v)
    except ValueError:
        raise ValueError('%s 必须是数值，收到 %r' % (key, v))


def _msg(h, info):
    h._send_json(info)


def status(h, q):
    """只读：编辑器当前状态（免 token，供 SSE/巡检）"""
    _msg(h, {'ok': True, **h.node.map_editor.status()})


def active(h, q):
    """只读：nav2_map_server 该加载哪个前缀（clean 优先）"""
    base = q.get('base', [''])[0] or os.path.expanduser(
        '~/RobotCode/04_map/robot_map')
    cur = h.node.map_editor.active_prefix(base)
    _msg(h, {'ok': True, 'active': cur,
             'has_clean': cur.endswith('_clean')})


def load(h, q):
    """从 /map 抓一帧作为编辑底图"""
    try:
        ok, msg = h.node.map_editor.load_from_topic(h.node, timeout=10.0)
    except Exception as e:
        _msg(h, {'ok': False, 'msg': '载入失败：%s' % e})
        return
    _msg(h, {'ok': ok, 'msg': msg, **h.node.map_editor.status()})


def info(h, q):
    """地图几何信息（画布换算用）"""
    ed = h.node.map_editor
    if ed.grid is None:
        _msg(h, {'ok': False, 'msg': '尚未载入地图'})
        return
    st = ed.status()
    _msg(h, {'ok': True, 'w': st['w'], 'h': st['h'],
             'resolution': st['resolution'], 'origin': st['origin']})


def image(h, q):
    """把当前栅格渲染成 PNG（白=空闲 灰=未知 黑=障碍），画布直接贴"""
    ed = h.node.map_editor
    if ed.grid is None:
        h._send(404, 'text/plain', b'not loaded')
        return
    with ed.lock:
        grid = ed.grid.copy()
    img = np.full(grid.shape, 205, np.uint8)      # 未知 灰
    img[grid == 0] = 254                          # 空闲 白
    img[grid >= 65] = 0                           # 障碍 黑
    img = np.flipud(img)                          # 原点在左下 → 图片行从上到下
    ok, buf = cv2.imencode('.png', img)
    if not ok:
        h._send(500, 'text/plain', b'encode failed')
        return
    h._send(200, 'image/png', buf.tobytes(),
            {'Cache-Control': 'no-store'})


def stroke(h, q):
    """一笔笔刷：?mode=erase|wall&radius=1&pts=x1,y1;x2,y2;...

    pts 是**世界坐标**（米），服务端按 origin/resolution 换算到格子。
    一笔提交一次（松手时发），支持撤销整笔。
    """
    try:
        mode = q.get('mode', [''])[0]
        radius = int(_num(q, 'radius', 1) or 1)
        raw = q.get('pts', [''])[0]
        pts = []
        for item in raw.split(';'):
            item = item.strip()
            if not item:
                continue
            xs, ys = item.split(',')
            pts.append((float(xs), float(ys)))
        if not pts:
            raise ValueError('pts 为空')

        ed = h.node.map_editor
        if ed.grid is None:
            _msg(h, {'ok': False, 'msg': '✗ 尚未载入地图（先点「抓取当前地图」）',
                     **ed.status()})
            return
        cells = []
        for x, y in pts:
            c = ed.world_to_cell(x, y)
            if c is not None:
                cells.append(c)
        if not cells:
            _msg(h, {'ok': False, 'msg': '所有点都在地图外', **ed.status()})
            return
        ok, msg = ed.apply_stroke(cells, mode, radius)
    except (TypeError, ValueError) as e:
        _msg(h, {'ok': False, 'msg': '参数错误：%s' % e})
        return
    _msg(h, {'ok': ok, 'msg': msg, **h.node.map_editor.status()})


def undo(h, q):
    ok, msg = h.node.map_editor.undo()
    _msg(h, {'ok': ok, 'msg': msg, **h.node.map_editor.status()})


def reset(h, q):
    ok, msg = h.node.map_editor.reset()
    _msg(h, {'ok': ok, 'msg': msg, **h.node.map_editor.status()})


def save(h, q):
    """另存新版本（原子写）；**拒绝覆盖 robot_map.***"""
    prefix = q.get('prefix', [''])[0] or DEFAULT_EXPORT
    prefix = os.path.expanduser(prefix)
    if os.path.basename(prefix) in PROTECTED:
        _msg(h, {'ok': False,
                 'msg': '✗ 拒绝：不得覆盖原始地图 %s.*（P2 另存不覆盖原则）'
                        % os.path.basename(prefix)})
        return
    try:
        ok, msg, files = h.node.map_editor.export(prefix)
    except Exception as e:
        _msg(h, {'ok': False, 'msg': '导出失败：%s' % e})
        return
    _msg(h, {'ok': ok, 'msg': msg, 'files': files,
             **h.node.map_editor.status()})
