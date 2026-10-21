"""前沿探索路由（P2-B1）：启动/停止/存档/显式启用

薄壳：参数解析 → 转发给 `h.node.explore`（`ExploreRunner`）。
运动控制由 ExploreRunner 通过 nav2 action 下发，**不发 `/cmd_vel`**。

存档不切（与用户确认）：
    探索结果另存 `robot_map_v2`（已存在递增），**永不覆盖 `robot_map.*`、不自动激活**。
    要切必须显式调 `/explore/activate`（会先备份原图）。
"""
import os
import shutil

MAP_DIR = os.path.expanduser('~/RobotCode/04_map')
MAP_BASE = os.path.join(MAP_DIR, 'robot_map')
BACKUP_DIR = os.path.join(MAP_DIR, 'bak_activate')


def _msg(h, info):
    h._send_json(info)


def status(h, q):
    """只读：探索状态（免 token，供 SSE/巡检）"""
    _msg(h, {'ok': True, **h.node.explore.status()})


def image(h, q):
    """把**实时 /map** 渲染成 PNG（探索屏画布底图），与编辑屏同一套配色"""
    import numpy as np
    try:
        import cv2
    except ImportError:
        return h._send(500, 'text/plain', b'no cv2')
    m = getattr(h.node, 'raw_map_msg', None)
    if m is None:
        return h._send(404, 'text/plain', b'no map')
    grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    img = np.full(grid.shape, 205, np.uint8)      # 未知 灰
    img[grid == 0] = 254                          # 空闲 白
    img[grid >= 65] = 0                           # 障碍 黑
    img = np.flipud(img)                          # 原点在左下 → 图片行从上到下
    ok, buf = cv2.imencode('.png', img)
    if not ok:
        return h._send(500, 'text/plain', b'encode failed')
    h._send(200, 'image/png', buf.tobytes(), {'Cache-Control': 'no-store'})


def info(h, q):
    """实时地图几何（画布换算用）"""
    m = getattr(h.node, 'raw_map_msg', None)
    if m is None:
        return _msg(h, {'ok': False, 'msg': '还没有 /map'})
    _msg(h, {'ok': True, 'w': int(m.info.width), 'h': int(m.info.height),
             'resolution': round(float(m.info.resolution), 4),
             'origin': [round(float(m.info.origin.position.x), 3),
                        round(float(m.info.origin.position.y), 3)]})


def go(h, q):
    """启动探索：?mode=continue|fresh（默认 continue = 续建已有图）"""
    mode = q.get('mode', ['continue'])[0]
    try:
        _msg(h, h.node.explore.go(mode))
    except Exception as e:
        _msg(h, {'ok': False, 'msg': '启动失败：%s' % e})


def cancel(h, q):
    _msg(h, h.node.explore.cancel())


def save(h, q):
    """手动存档（存档不切）：另存新版本前缀"""
    from save_map import serialize_posegraph
    try:
        for i in range(2, 50):
            prefix = '%s_v%d' % (MAP_BASE, i)
            if not os.path.exists(prefix + '.posegraph'):
                ok, msg = serialize_posegraph(h.node, prefix, timeout=30.0, spin=False)
                _msg(h, {'ok': ok, 'msg': msg + '（已存档，未激活）',
                         'saved_as': prefix if ok else None,
                         **h.node.explore.status()})
                return
        _msg(h, {'ok': False, 'msg': '版本号用尽（robot_map_v2.._v49 都已存在）'})
    except Exception as e:
        _msg(h, {'ok': False, 'msg': '保存失败：%s' % e})


def activate(h, q):
    """显式把某个存档版本切为当前地图（先备份原图）

    ?prefix=/path/robot_map_v2   —— 缺省用 explore 最近一次的 saved_as
    ⚠️ 这是**唯一**会写 robot_map.* 的入口，所以必须显式调用 + 前端二次确认。
    """
    import time
    prefix = q.get('prefix', [''])[0] or (h.node.explore.status().get('saved_as') or '')
    prefix = os.path.expanduser(prefix)
    if not prefix:
        _msg(h, {'ok': False, 'msg': '✗ 没有可启用的存档（prefix 为空）'})
        return
    if os.path.basename(prefix) == 'robot_map':
        _msg(h, {'ok': False, 'msg': '✗ 无需启用：已是当前地图'})
        return
    missing = [e for e in ('.pgm', '.yaml', '.posegraph', '.data')
               if not os.path.isfile(prefix + e)]
    if missing:
        _msg(h, {'ok': False, 'msg': '✗ 存档不完整，缺 %s' % ' '.join(missing)})
        return

    try:
        os.makedirs(BACKUP_DIR, exist_ok=True)
        stamp = time.strftime('%m%d_%H%M%S')
        backed = []
        for e in ('.pgm', '.yaml', '.posegraph', '.data'):
            src = MAP_BASE + e
            if os.path.isfile(src):
                shutil.copy2(src, os.path.join(BACKUP_DIR, 'robot_map_%s%s' % (stamp, e)))
                backed.append(e)
        for e in ('.pgm', '.yaml', '.posegraph', '.data'):
            shutil.copy2(prefix + e, MAP_BASE + e)
    except Exception as e:
        _msg(h, {'ok': False, 'msg': '✗ 切换失败：%s' % e})
        return

    _msg(h, {'ok': True,
             'msg': '✓ 已启用 %s（原图已备份到 %s）'
                    % (os.path.basename(prefix), os.path.basename(BACKUP_DIR)),
             'active': MAP_BASE, 'backup': BACKUP_DIR,
             'backed': backed, **h.node.explore.status()})
