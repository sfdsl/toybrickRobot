"""手机控制台服务（Task 3 阶段：路由表驱动 + www/ 静态前端）

板端用法：cd ~/RobotCode/01_Project/03_SLAM && python3 -m robot_ui.app
命令行参数与原 phone_teleop.py 完全一致。
⚠ 部署：www/ 目录必须随 robot_ui/ 一起同步到板端——首页已无内嵌兜底，
  缺 www/ 会直接 404。
"""
import os as _os, sys as _sys
_sys.path.insert(0, _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))
import argparse
import json
import secrets
import socket
import threading
import time
import os
import sys
# node_teleop 的守卫导入先触发：环境未 source 时能给出友好中文提示（否则裸 import cv2 会甩 traceback）
from robot_ui.node_teleop import (PhoneTeleop, CMD_TIMEOUT, STREAM_FPS, CAM_FPS,
                                  RANGE_MIN, RANGE_MAX, PAN_MAX)
import cv2
import rclpy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse
import goal_nav as gn
from save_map import DEFAULT_SAVE_DIR
import robot_ui.routes_teleop as routes_teleop
import robot_ui.routes_nav as routes_nav
import robot_ui.routes_patrol as routes_patrol
import robot_ui.routes_places as routes_places
import robot_ui.routes_task as routes_task
from robot_ui import state_store

# 静态文件目录（/ 与真实文件由 serve_static 提供）
WWW_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'www')

MIME = {'.html': 'text/html; charset=utf-8', '.css': 'text/css; charset=utf-8',
        '.js': 'application/javascript; charset=utf-8', '.svg': 'image/svg+xml',
        '.png': 'image/png', '.jpg': 'image/jpeg', '.ico': 'image/x-icon',
        '.json': 'application/json; charset=utf-8'}

# 表驱动路由：key = (方法, 路径)，value = fn(handler, query)
ROUTES = {
    ('GET', '/cmd'):        routes_teleop.cmd,
    ('GET', '/stop'):       routes_teleop.stop,
    ('GET', '/view'):       routes_teleop.view,
    ('GET', '/savemap'):    routes_teleop.savemap,
    ('GET', '/state'):      routes_teleop.state,
    ('GET', '/features'):   routes_teleop.features,
    ('GET', '/goal'):       routes_nav.goal,
    ('GET', '/goal/home'):  routes_nav.goal_home,
    ('GET', '/goal/clear'): routes_nav.goal_clear,
    ('GET', '/wp/add'):     routes_nav.wp_add,
    ('GET', '/wp/go'):      routes_nav.wp_go,
    ('GET', '/wp/clear'):   routes_nav.wp_clear,
    # 巡逻（Task 6，屏2/屏3）：与旧 /wp/* 并存，走 PatrolRunner 顺序逐点
    ('GET', '/patrol/load'):    routes_patrol.load,
    ('GET', '/patrol/points'):  routes_patrol.points,
    ('GET', '/patrol/go'):      routes_patrol.go,
    ('GET', '/patrol/pause'):   routes_patrol.pause,
    ('GET', '/patrol/resume'):  routes_patrol.resume,
    ('GET', '/patrol/skip'):    routes_patrol.skip,
    ('GET', '/patrol/cancel'):  routes_patrol.cancel,
    ('GET', '/places/list'):   routes_places.listing,
    ('GET', '/places/add'):    routes_places.add,
    ('GET', '/places/delete'): routes_places.delete,
    ('GET', '/places/go'):     routes_places.go,
    ('GET', '/task/request'):  routes_task.request,
}


class Handler(BaseHTTPRequestHandler):
    node = None          # 由 main() 注入
    token = ''           # 鉴权 token（main() 注入；空串 = 关闭鉴权，全部放行）
    frame = b''          # 最近一帧 JPEG（渲染线程更新）
    frame_lock = threading.Lock()
    protocol_version = 'HTTP/1.1'

    def log_message(self, fmt, *args):
        pass             # 静音访问日志（1 秒几十条会刷屏）

    def _send(self, code, ctype, body, extra=None):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, info):
        body = json.dumps(info, ensure_ascii=False).encode('utf-8')
        self._send(200, 'application/json; charset=utf-8', body,
                   {'Cache-Control': 'no-store'})

    def serve_static(self, path):
        """静态文件：/ → www/index.html，其余按 WWW_DIR 下的真实文件提供

        .html/.js/.css 文本文件下发前做 __K__ 占位符替换（前端借它拿到鉴权 token；
        对 .css 替换无害——里面不会出现该字面量）。
        """
        name = 'index.html' if path in ('/', '') else path.lstrip('/')
        fp = os.path.normpath(os.path.join(WWW_DIR, name))
        if not fp.startswith(WWW_DIR + os.sep) or not os.path.isfile(fp):
            return self._send(404, 'text/plain; charset=utf-8', 'not found'.encode('utf-8'))
        ext = os.path.splitext(fp)[1].lower()
        with open(fp, 'rb') as f:
            body = f.read()
        if ext in ('.html', '.js', '.css'):
            body = body.replace(b'__K__', Handler.token.encode())
        self._send(200, MIME.get(ext, 'application/octet-stream'), body)

    def serve_stream(self):
        """MJPEG 长连接（原 /stream.mjpg 分支体逐字平移）"""
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            while True:
                with Handler.frame_lock:
                    data = Handler.frame
                if data:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(data)).encode()
                                     + b'\r\n\r\n' + data + b'\r\n')
                time.sleep(1.0 / STREAM_FPS)
        except OSError:
            pass            # 断连/半开超时（BrokenPipe、ConnectionReset、ETIMEDOUT 都是 OSError 子类）

    def serve_cam_stream(self):
        """相机画框流 /cam.mjpg（《13》P3）：直接搬运 rknn_infer 的 /image_det JPEG。

        与地图流同款 MJPEG 协议；rknn_infer 端已按 hud_fps(默认10fps) 节流，
        这里只是把最新一帧按 ~10fps 重发，不做任何解码/渲染。
        """
        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            while True:
                data = self.node.cam_jpeg()
                if data:
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     b'Content-Length: ' + str(len(data)).encode()
                                     + b'\r\n\r\n' + data + b'\r\n')
                time.sleep(1.0 / CAM_FPS)
        except OSError:
            pass

    def serve_events(self):
        """GET /events：SSE 单向推送 state（0.5s 一拍；ThreadingHTTPServer 每连接一线程）"""
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try:
            while True:
                data = json.dumps(self.node.state(), ensure_ascii=False).encode('utf-8')
                self.wfile.write(b'data: ' + data + b'\n\n')
                time.sleep(0.5)
        except OSError:
            pass            # 同上：断连统一兜底，避免半开连接挂线程 + 噪音 traceback

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path == '/stream.mjpg':
            return self.serve_stream()
        if u.path == '/cam.mjpg':
            return self.serve_cam_stream()
        if u.path == '/events':
            return self.serve_events()
        h = ROUTES.get(('GET', u.path))
        if h:
            # 鉴权：token 开启时，敏感路由必须带 k=<token>。
            # 白名单（只读或本就免敏）：/state /patrol/points /features；
            # /stream.mjpg、/events、静态文件在上面分支已放行，不经过这里。
            PUBLIC = ('/state', '/patrol/points', '/features', '/places/list')
            if Handler.token and u.path not in PUBLIC and q.get('k', [''])[0] != Handler.token:
                return self._send(403, 'application/json; charset=utf-8',
                                  json.dumps({'ok': False, 'msg': '✗ 鉴权失败：地址缺 k= 参数（用启动时打印的完整地址）'},
                                             ensure_ascii=False).encode('utf-8'))
            return h(self, q)
        return self.serve_static(u.path)


def spin_forever(node):
    """rclpy.spin 的容错版：单个回调抛异常不再让整个 UI 变"僵尸"

    教训（P35）：`_wp_feedback` 取了不存在的反馈字段 → 异常从 executor 冒到 spin 线程 →
    线程退出 → /odom、/map、/tf 全部不再处理，页面上的车永远停在异常发生那一刻的位置
    （车还在跑，但画面不动）。这里把异常吃掉并续 spin，异常最多打一条日志。
    """
    while rclpy.ok():
        try:
            rclpy.spin(node)
        except Exception as e:
            print('[spin] 回调异常已忽略，继续 spin：%r' % (e,), flush=True)
            time.sleep(0.5)


def render_loop(node, size):
    """后台线程：把地图渲染成 JPEG（手机端直接从缓存取，避免并发渲染）

    顺带把"已设置的目标点"画在画面上（红圈 + 十字 + 车→目标连线）。
    缩放/平移由 node 的视图状态决定（服务端重渲染，放大不糊）。
    """
    while rclpy.ok():
        try:
            img = node.render(size=size, view_range=node.view_range_cur,
                              offset=node.view_offset())
            o = node._world_px(0.0, 0.0, size)  # 地图原点 = 建图起点（「回原点」的目标）
            if o is not None:
                cv2.circle(img, o, 6, (0, 170, 255), 2)
                cv2.drawMarker(img, o, (0, 170, 255), cv2.MARKER_CROSS, 12, 2)
                cv2.putText(img, 'origin', (o[0] + 9, o[1] + 17),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 170, 255), 1)
            g = node.goal_px(size)
            # P34 多点导航：画航点队列（青色点 + 编号 + 顺序连线）
            q = list(node.wp_queue)
            if q:
                prev = node._world_px(q[0][0], q[0][1], size)
                for i, (wx, wy) in enumerate(q):
                    p = node._world_px(wx, wy, size)
                    if p is None:
                        prev = None
                        continue
                    cv2.circle(img, p, 5, (255, 160, 0), 2)
                    cv2.putText(img, str(i + 1), (p[0] + 7, p[1] - 7),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 160, 0), 1)
                    if prev is not None and i > 0:
                        cv2.line(img, prev, p, (255, 160, 0), 1)
                    prev = p
            if g is not None:
                c = node.car_px(size)           # 平移后车不在画面中心 → 从车的位置连线
                if c is not None:
                    cv2.line(img, c, g, (0, 0, 255), 1)
                cv2.circle(img, g, 14, (0, 0, 255), 2)
                cv2.drawMarker(img, g, (0, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 2)
                cv2.putText(img, 'goal', (g[0] + 12, g[1] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            ok, buf = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            if ok:
                with Handler.frame_lock:
                    Handler.frame = buf.tobytes()
        except Exception as e:
            node.get_logger().error('渲染失败：%s' % e, throttle_duration_sec=5.0)
        time.sleep(1.0 / STREAM_FPS)


def local_ip():
    """取本机在局域网里的 IP（用于打印手机要打开的地址）"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description='手机遥控小车（网页版，板子当服务器）')
    ap.add_argument('-p', '--port', type=int, default=8080, help='HTTP 端口（默认 8080）')
    ap.add_argument('-r', '--range', type=float, default=8.0,
                    help='初始视野米数（默认 8；手机上可捏合缩放 1~20）')
    ap.add_argument('--size', type=int, default=480, help='推流画面像素边长（默认 480）')
    ap.add_argument('-v', '--vmax', type=float, default=0.20, help='前进速度上限 m/s（默认 0.20）')
    ap.add_argument('-w', '--wmax', type=float, default=1.00, help='转向速度上限 rad/s（默认 1.00）')
    ap.add_argument('-t', '--topic', default='/cmd_vel',
                    help='速度话题（默认 /cmd_vel；自测时可换成测试话题，不动真车）')
    ap.add_argument('-g', '--goal-topic', default='/goal_pose',
                    help='目标点话题（默认 /goal_pose；自测可换成测试话题）')
    ap.add_argument('--no-precheck', action='store_true',
                    help='关闭点选预检（默认开启：点选/回原点前用 nav 同源代码本地试算，'
                         '不可达就不下发目标；预检按默认参数）')
    ap.add_argument('--map', default=gn.MAP_DEFAULT,
                    help='预检用的地图前缀（默认 %s）' % gn.MAP_DEFAULT)
    ap.add_argument('-s', '--save-dir', default=DEFAULT_SAVE_DIR,
                    help='保存地图的目录（默认 ~/RobotCode/04_map）')
    ap.add_argument('--no-auth', action='store_true',
                    help='关闭 token 鉴权（默认开启：/cmd /goal 等敏感接口需要 k=<token> 参数）')
    args = ap.parse_args()

    # 鉴权 token：16 位 hex（8 字节），启动时生成；--no-auth 关闭（置空 = 全放行）
    TOKEN = '' if args.no_auth else secrets.token_hex(8)
    Handler.token = TOKEN

    # 功能注册表预置（hub 屏4 动态渲染用，须在 rclpy.init() 之前；顺序 A1→B6 与 hub 页一致）
    state_store.register_feature('手机点图导航', '已具备', href='/')
    state_store.register_feature('多点巡逻', '已具备', href='/nav.html')
    state_store.register_feature('语义标注地图', '已具备', detail='地点名称与地图版本绑定')
    state_store.register_feature('地图编辑器', '待做')
    state_store.register_feature('RKNN视觉', '进行中')
    state_store.register_feature('语音指令', '待做')
    state_store.register_feature('rosbag', '已具备')
    state_store.register_feature('状态灯屏按键', '进行中')
    state_store.register_feature('电量温度监控', '进行中', detail='电量已接入；温度待标定')
    state_store.register_feature('自主探索建图', '进行中')
    state_store.register_feature('动态避障', '待做')
    state_store.register_feature('目标跟随', '待做')
    state_store.register_feature('ArUco停靠', '待做')
    state_store.register_feature('自动回充', '待做')
    state_store.register_feature('语义地图', '待做')

    rclpy.init()
    node = PhoneTeleop(args.range, args.vmax, args.wmax, args.topic,
                       os.path.expanduser(args.save_dir), args.goal_topic,
                       precheck=not args.no_precheck, map_prefix=args.map)
    Handler.node = node

    threading.Thread(target=spin_forever, args=(node,), daemon=True).start()
    threading.Thread(target=node.watchdog, daemon=True).start()
    threading.Thread(target=render_loop, args=(node, args.size), daemon=True).start()

    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    srv.daemon_threads = True
    ip = local_ip()
    kq = ('?k=' + TOKEN) if TOKEN else ''      # 手机打开的地址直接带上 k（免手动拼参数）
    print('=' * 56)
    print('手机遥控已启动（手机需与小车的 WiFi 在同一网络）')
    if TOKEN:
        print('  鉴权 token：k=%s（--no-auth 可关闭；手机直接打开下方带 k 的地址）' % TOKEN)
    print('  手机浏览器打开： http://%s:%d/%s' % (ip, args.port, kq))
    print('  本机自测：       http://127.0.0.1:%d/%s' % (args.port, kq))
    print('  速度上限：前进 %.2f m/s ｜ 转向 %.2f rad/s（网页上还能再调小）'
          % (args.vmax, args.wmax))
    print('  地图手势：点一下=设目标 ｜ 拖动=平移 ｜ 捏合=缩放(1~20m) ｜ 双击=复位')
    print('  保存地图：网页右上角「保存地图」按钮 → %s' % node.save_dir)
    print('  点选目标：地图上点一下 → 发 %s（要真的开过去需另开终端跑  nav --listen）'
          % args.goal_topic)
    print('  Ctrl-C 退出（退出会自动发零速度）')
    print('=' * 56)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print('\n收到 Ctrl-C，退出中 ...')
    finally:
        node.set_cmd(0.0, 0.0)
        srv.shutdown()
        srv.server_close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
