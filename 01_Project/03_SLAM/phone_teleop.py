#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""手机遥控小车（板子当服务器，手机浏览器直接开网页）—— 零 App、零额外依赖

原理：本机起一个 HTTP 服务，网页里的虚拟摇杆把速度发回来，本节点转成 /cmd_vel。
      地图画面用 /map 渲染成 MJPEG 推给浏览器，手机上能边遥控边看建图效果。
      页面右上角还有「保存地图」按钮：一键把当前 SLAM 图存到
      ~/RobotCode/04_map/（文件名自动带时间戳，含 .pgm/.yaml + .posegraph/.data）。

只依赖：Python 标准库（http.server）+ rclpy + OpenCV（本机已有）。

订阅 /map（transient_local）、/tf（画车位置）、/odom（显示实际速度）
发布 /cmd_vel

用法：
    python3 phone_teleop.py                 # 端口 8080，视野 8 米
    python3 phone_teleop.py -p 9000 -r 12   # 换端口 / 视野
    python3 phone_teleop.py -s /tmp/maps    # 换地图保存目录（默认 ~/RobotCode/04_map）
    python3 phone_teleop.py -t /test_cmd_vel  # 换速度话题（自测用，不动真车）
手机：与小车连**同一个 WiFi**，浏览器打开  http://<小车IP>:8080/
      （本机 IP 用别名 myip 查看；启动时终端也会打印）
桌面：同一页面上也能用方向键（↑↓ 前后，←→ 转向，空格 停）

安全（三层）：
    1. 手指离开摇杆 / 页面断开 → 网页立刻发零速度
    2. 本节点看门狗：0.6 s 没收到新指令 → 自动发零速度并停车
    3. 底盘节点自带看门狗：0.5 s 没收到 /cmd_vel → 自动停车
    * 速度上限服务端强制裁剪（-v/-w 可调）；**不发横移速度 vy**（麦轮横移时里程计不可信）
    * 「保存地图」会先自动停车再存（存图时车不动，地图才是"那一刻"的）
    * 服务只监听局域网、无鉴权——用完记得 Ctrl-C 关掉
"""
import argparse
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

try:
    import cv2
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
except ImportError as e:
    raise SystemExit(
        '导入模块失败：%s\n提示：先 source 环境（或用 ~/.bash_aliases 里的别名 phone）\n'
        '  source /opt/ros2-foxy/install/setup.bash\n'
        '  source ~/RobotCode/ros2_ws/install/setup.bash' % e)

from map_viewer import MapViewer      # 复用地图订阅 + 渲染（同目录）
from save_map import DEFAULT_SAVE_DIR, save_pgm_yaml, serialize_posegraph

CMD_TIMEOUT = 0.6                     # 看门狗：多久没收到指令就发零速度
STREAM_FPS = 8                        # 推给手机的帧率

PAGE = """<!DOCTYPE html>
<html lang="zh"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>小车遥控</title>
<style>
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  html,body { margin:0; height:100%; background:#111; color:#eee;
              font-family:-apple-system,"Noto Sans CJK SC",sans-serif; overflow:hidden; }
  #wrap { display:flex; flex-direction:column; height:100%; }
  #mapbox { position:relative; flex:1 1 auto; min-height:30%; overflow:hidden; }
  #map { width:100%; height:100%; object-fit:contain; display:block; }
  #hud { position:absolute; left:8px; top:8px; font-size:12px; line-height:1.5;
         background:rgba(0,0,0,.45); padding:6px 8px; border-radius:6px; }
  #hud b { color:#7fd1ff; font-weight:600; }
  .dot { display:inline-block; width:8px; height:8px; border-radius:50%; background:#666; }
  .ok { background:#3fbf5f; } .bad { background:#e33; }
  #save { position:absolute; right:8px; top:8px; padding:10px 14px; font-size:15px;
          font-weight:700; color:#fff; background:#2d6cdf; border:0; border-radius:10px; }
  #save:active { background:#3f86f5; }
  #save[disabled] { background:#555; }
  #toast { position:absolute; left:50%; transform:translateX(-50%); bottom:12px;
           max-width:92%; padding:8px 12px; font-size:13px; line-height:1.4;
           background:rgba(0,0,0,.78); border-radius:8px; display:none; }
  #padbox { flex:0 0 auto; padding:10px 12px 14px; background:#181818; }
  #pad { position:relative; width:100%; height:34vh; min-height:180px; max-height:300px;
         background:#222; border:1px solid #333; border-radius:14px; touch-action:none; }
  #knob { position:absolute; width:84px; height:84px; margin:-42px 0 0 -42px;
          left:50%; top:50%; border-radius:50%; background:#3a6ea5;
          border:2px solid #7fb2e5; transition:background .1s; }
  #knob.on { background:#4f8fd0; }
  #row { display:flex; gap:10px; align-items:center; margin-top:10px; }
  #stop { flex:0 0 34%; height:64px; font-size:22px; font-weight:700; color:#fff;
          background:#c0392b; border:0; border-radius:12px; }
  #stop:active { background:#e74c3c; }
  #sliders { flex:1 1 auto; font-size:12px; color:#bbb; }
  #sliders input { width:100%; }
  #hint { font-size:11px; color:#777; margin-top:6px; text-align:center; }
</style></head>
<body><div id="wrap">
  <div id="mapbox">
    <img id="map" src="/stream.mjpg" alt="map">
    <div id="hud">
      <span id="dot" class="dot"></span> <span id="conn">连接中…</span><br>
      位姿 <b id="pose">-</b><br>
      速度 <b id="vel">-</b>
    </div>
    <button id="save">保存地图</button>
    <div id="toast"></div>
  </div>
  <div id="padbox">
    <div id="pad"><div id="knob"></div></div>
    <div id="row">
      <button id="stop">停 止</button>
      <div id="sliders">
        前进上限 <span id="vl">0.15</span> m/s
        <input id="vs" type="range" min="5" max="30" value="15">
        转向上限 <span id="wl">0.60</span> rad/s
        <input id="ws" type="range" min="20" max="150" value="60">
      </div>
    </div>
    <div id="hint">拖动圆盘：上=前进，左右=转向 ｜ 松手即停 ｜ 右上角可保存地图 ｜ 桌面可用方向键</div>
  </div>
</div>
<script>
const pad=document.getElementById('pad'), knob=document.getElementById('knob');
const stopBtn=document.getElementById('stop');
const vs=document.getElementById('vs'), ws=document.getElementById('ws');
const saveBtn=document.getElementById('save'), toast=document.getElementById('toast');
let vmax=0.15, wmax=0.60, timer=null, vx=0, wz=0, active=false;

function cfg(){ vmax=vs.value/100; wmax=ws.value/100;
  document.getElementById('vl').textContent=vmax.toFixed(2);
  document.getElementById('wl').textContent=wmax.toFixed(2); }
vs.oninput=cfg; ws.oninput=cfg; cfg();

function send(){ fetch('/cmd?vx='+vx.toFixed(3)+'&wz='+wz.toFixed(3)).catch(()=>{}); }
function loop(){ if(!timer) timer=setInterval(send,100); }
function halt(){ vx=0; wz=0; send(); if(timer){clearInterval(timer); timer=null;} }
function setKnob(dx,dy,use){ const r=pad.clientWidth/2-52, R=Math.min(r,pad.clientHeight/2-52);
  const x=Math.max(-R,Math.min(R,dx)), y=Math.max(-R,Math.min(R,dy));
  knob.style.transform='translate('+x+'px,'+y+'px)';
  vx=(-y/R)*vmax; wz=(-x/R)*wmax;
  if(use){ loop(); send(); } knob.classList.toggle('on', !!use); }

function onMove(ev){ const r=pad.getBoundingClientRect();
  setKnob(ev.clientX-(r.left+r.width/2), ev.clientY-(r.top+r.height/2), true); }
pad.addEventListener('pointerdown', e=>{ active=true; pad.setPointerCapture(e.pointerId); onMove(e); });
pad.addEventListener('pointermove', e=>{ if(active) onMove(e); });
['pointerup','pointercancel'].forEach(t=>pad.addEventListener(t, e=>{
  active=false; setKnob(0,0,false); halt(); }));
stopBtn.onclick=()=>{ active=false; setKnob(0,0,false); halt(); };

// 保存地图：先停车，再让板子把 .pgm/.yaml + .posegraph/.data 存到 04_map（名字带时间戳）
function showToast(msg, ms){ toast.textContent=msg; toast.style.display='block';
  clearTimeout(showToast.t); showToast.t=setTimeout(()=>toast.style.display='none', ms||5000); }
saveBtn.onclick=()=>{
  active=false; setKnob(0,0,false); halt();
  saveBtn.disabled=true; saveBtn.textContent='保存中…';
  fetch('/savemap').then(r=>r.json()).then(s=>{
    showToast((s.ok?'✓ 已保存：':'✗ 保存失败：')+(s.msg||''), s.ok?7000:9000);
  }).catch(()=>showToast('✗ 保存失败：连不上服务（板子上的 phone 还在跑吗？）',8000))
    .finally(()=>{ saveBtn.disabled=false; saveBtn.textContent='保存地图'; });
};

// 桌面调试用方向键
const keys={};
document.addEventListener('keydown', e=>{ keys[e.key]=true; applyKeys(); if(e.key===' ') halt(); });
document.addEventListener('keyup',   e=>{ keys[e.key]=false; applyKeys(); });
function applyKeys(){ if(active) return;
  const dv=(keys.ArrowUp?1:0)-(keys.ArrowDown?1:0), dw=(keys.ArrowLeft?1:0)-(keys.ArrowRight?1:0);
  if(dv||dw){ vx=dv*vmax*0.8; wz=dw*wmax*0.8; loop(); send(); }
  else { vx=0; wz=0; if(timer){clearInterval(timer); timer=null;} send(); } }

setInterval(()=>{ fetch('/state').then(r=>r.json()).then(s=>{
  const d=document.getElementById('dot'), c=document.getElementById('conn');
  d.className='dot '+(s.cmd_age<1?'ok':'bad'); c.textContent=s.cmd_age<1?'已连接':'看门狗待机';
  document.getElementById('pose').textContent=s.pose||'-';
  document.getElementById('vel').textContent=s.vel||'-';
}).catch(()=>{ document.getElementById('dot').className='dot bad';
               document.getElementById('conn').textContent='断开'; }); }, 400);
</script></body></html>
"""


class PhoneTeleop(MapViewer):
    """在地图查看器基础上：发 /cmd_vel、保存地图、提供网页服务所需的状态"""

    def __init__(self, view_range, vmax, wmax, topic='/cmd_vel',
                 save_dir=DEFAULT_SAVE_DIR):
        super().__init__(view_range)
        self.topic = topic
        self.vmax = vmax
        self.wmax = wmax
        self.save_dir = save_dir      # 保存地图的目录（默认 ~/RobotCode/04_map）
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)         # 最近一次下发的 (vx, wz)
        self.last_cmd_t = 0.0         # 最近一次收到网页指令的时刻
        self.n_cmd = 0
        self.last_save = None         # 最近一次保存的文件名前缀
        self.odom_vel = None          # (vx, wz) 底盘回报的实际速度
        self.pub = self.create_publisher(Twist, self.topic, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

    def on_odom(self, msg):
        self.odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def set_cmd(self, vx, wz):
        """裁剪并发布速度；返回实际下发值"""
        vx = max(-self.vmax, min(self.vmax, vx))
        wz = max(-self.wmax, min(self.wmax, wz))
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        with self.lock:
            self.pub.publish(t)
            self.cmd = (vx, wz)
            self.last_cmd_t = time.time()
            self.n_cmd += 1
        return vx, wz

    def save_map(self):
        """保存当前 SLAM 图（PGM/YAML + 建图序列），文件名带时间戳，存到 self.save_dir

        返回给网页的 dict：{'ok': bool, 'msg': str, 'name': str, 'dir': str}
        """
        m = self.map_msg
        if m is None:
            return {'ok': False, 'msg': '还没收到 /map —— SLAM 没在运行？'}

        try:
            os.makedirs(self.save_dir, exist_ok=True)
        except OSError as e:
            return {'ok': False, 'msg': '无法创建目录 %s（%s）' % (self.save_dir, e)}

        name = time.strftime('map_%Y%m%d_%H%M%S')
        prefix = os.path.join(self.save_dir, name)
        written, notes = [], []

        try:
            written += save_pgm_yaml(m, prefix)
            notes.append('pgm+yaml(%dx%d)' % (m.info.width, m.info.height))
        except Exception as e:
            notes.append('pgm 失败：%s' % e)

        ok_pg, info = serialize_posegraph(self, prefix, timeout=30.0, spin=False)
        if ok_pg:
            written += [prefix + '.posegraph', prefix + '.data']
            notes.append('posegraph')
        else:
            notes.append('序列失败：%s' % info)

        sizes = ' ｜ '.join('%s %.0fKB' % (os.path.basename(p), os.path.getsize(p) / 1024.0)
                            for p in written if os.path.exists(p))
        self.get_logger().info('保存地图 → %s（%s）' % (prefix, ' + '.join(notes)))
        self.last_save = name
        return {'ok': bool(written), 'msg': '%s ｜ %s' % (name, sizes or '未写出文件'),
                'name': name, 'dir': self.save_dir}

    def watchdog(self):
        """超时未收到网页指令 → 主动发零速度（第 2 层保险）"""
        while rclpy.ok():
            time.sleep(0.1)
            with self.lock:
                stale = self.last_cmd_t > 0 and time.time() - self.last_cmd_t > CMD_TIMEOUT
                moving = abs(self.cmd[0]) > 1e-6 or abs(self.cmd[1]) > 1e-6
            if stale and moving:
                self.set_cmd(0.0, 0.0)
                self.get_logger().warn('网页指令超时 %.1fs，已自动停车' % CMD_TIMEOUT)

    def state(self):
        pose = self.robot_pose()
        with self.lock:
            cmd_age = time.time() - self.last_cmd_t if self.last_cmd_t else 1e9
            vx, wz = self.cmd
            n = self.n_cmd
        m = self.map_msg
        return {
            'pose': ('x=%.2f y=%.2f yaw=%.0f°' % (pose[0], pose[1], math.degrees(pose[2])))
                    if pose else None,
            'vel': ('cmd=%.2f m/s %.2f rad/s ｜ 实际=%.2f m/s %.2f rad/s'
                    % (vx, wz, self.odom_vel[0], self.odom_vel[1])) if self.odom_vel
                   else ('cmd=%.2f m/s %.2f rad/s' % (vx, wz)),
            'cmd_age': round(cmd_age, 2),
            'n_cmd': n,
            'map': ('%dx%d @%.3fm' % (m.info.width, m.info.height, m.info.resolution))
                   if m else None,
            'last_save': self.last_save,
        }


class Handler(BaseHTTPRequestHandler):
    node = None          # 由 main() 注入
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

    def do_GET(self):
        u = urlparse(self.path)
        q = parse_qs(u.query)
        if u.path in ('/', '/index.html'):
            self._send(200, 'text/html; charset=utf-8', PAGE.encode('utf-8'))
        elif u.path == '/cmd':
            try:
                vx = float(q.get('vx', ['0'])[0])
                wz = float(q.get('wz', ['0'])[0])
                vx, wz = self.node.set_cmd(vx, wz)
            except (TypeError, ValueError):
                self._send(400, 'text/plain', b'bad args')
                return
            self._send(200, 'text/plain', ('%+.2f %+.2f' % (vx, wz)).encode())
        elif u.path == '/stop':
            self.node.set_cmd(0.0, 0.0)
            self._send(200, 'text/plain', b'stopped')
        elif u.path == '/savemap':
            # 保存地图（会阻塞几秒：等 serialize_map 服务写完文件）
            try:
                info = self.node.save_map()
            except Exception as e:
                info = {'ok': False, 'msg': '保存异常：%s' % e}
            body = json.dumps(info, ensure_ascii=False).encode('utf-8')
            self._send(200, 'application/json; charset=utf-8', body,
                       {'Cache-Control': 'no-store'})
        elif u.path == '/state':
            body = json.dumps(self.node.state(), ensure_ascii=False).encode('utf-8')
            self._send(200, 'application/json; charset=utf-8', body,
                       {'Cache-Control': 'no-store'})
        elif u.path == '/stream.mjpg':
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
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self._send(404, 'text/plain', b'not found')


def render_loop(node, size):
    """后台线程：把地图渲染成 JPEG（手机端直接从缓存取，避免并发渲染）"""
    while rclpy.ok():
        try:
            img = node.render(size=size)
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
    ap.add_argument('-r', '--range', type=float, default=8.0, help='地图视野米数（默认 8）')
    ap.add_argument('--size', type=int, default=480, help='推流画面像素边长（默认 480）')
    ap.add_argument('-v', '--vmax', type=float, default=0.20, help='前进速度上限 m/s（默认 0.20）')
    ap.add_argument('-w', '--wmax', type=float, default=1.00, help='转向速度上限 rad/s（默认 1.00）')
    ap.add_argument('-t', '--topic', default='/cmd_vel',
                    help='速度话题（默认 /cmd_vel；自测时可换成测试话题，不动真车）')
    ap.add_argument('-s', '--save-dir', default=DEFAULT_SAVE_DIR,
                    help='保存地图的目录（默认 ~/RobotCode/04_map）')
    args = ap.parse_args()

    rclpy.init()
    node = PhoneTeleop(args.range, args.vmax, args.wmax, args.topic,
                       os.path.expanduser(args.save_dir))
    Handler.node = node

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    threading.Thread(target=node.watchdog, daemon=True).start()
    threading.Thread(target=render_loop, args=(node, args.size), daemon=True).start()

    srv = ThreadingHTTPServer(('0.0.0.0', args.port), Handler)
    srv.daemon_threads = True
    ip = local_ip()
    print('=' * 56)
    print('手机遥控已启动（手机需与小车的 WiFi 在同一网络）')
    print('  手机浏览器打开： http://%s:%d/' % (ip, args.port))
    print('  本机自测：       http://127.0.0.1:%d/' % args.port)
    print('  速度上限：前进 %.2f m/s ｜ 转向 %.2f rad/s（网页上还能再调小）'
          % (args.vmax, args.wmax))
    print('  保存地图：网页右上角「保存地图」按钮 → %s' % node.save_dir)
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
