#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
双轮足 4关节位置控制 Web 界面
浏览器滑动条 -> /mcu/pos_cmd -> STM32 位置刚度控制

运行(需source ROS环境):
  ros2 run rl_policy pos_web_server   (或 python3 pos_web_server.py)
访问: http://<RDK_IP>:8000
"""
import json
import threading
import http.server
import socketserver

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

# 关节名称与安全范围 (rad)
JOINTS = [
    ("hip_R",  -1.5, 1.5),
    ("knee_R", -1.5, 1.5),
    ("hip_L",  -1.5, 1.5),
    ("knee_L", -1.5, 1.5),
]

DEFAULT_KP = 50.0
DEFAULT_KD = 1.0

# 当前目标位置 (rad)
current_pos = [0.0] * 4
current_kp = DEFAULT_KP
current_kd = DEFAULT_KD

pub = None  # rclpy publisher


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class PosCommandNode(Node):
    def __init__(self):
        super().__init__('pos_web_server')
        self.pub = self.create_publisher(Float32MultiArray, '/mcu/pos_cmd', 10)
        self.enable_pub = self.create_publisher(Bool, '/mcu/enable', 10)


def publish_pos():
    global pub
    if pub is None:
        return
    msg = Float32MultiArray()
    msg.data = list(current_pos) + [current_kp, current_kd]
    pub.publish(msg)


def publish_enable(en):
    global enable_pub
    if enable_pub is not None:
        msg = Bool()
        msg.data = en
        enable_pub.publish(msg)


HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>关节位置控制</title>
<style>
body{font-family:sans-serif;margin:30px;background:#1a1a2e;color:#eee}
.slider-row{margin:18px 0;display:flex;align-items:center;gap:15px}
.slider-row label{width:90px;font-weight:bold}
input[type=range]{width:420px}
.val{width:90px;text-align:right;font-family:monospace}
button{padding:10px 22px;margin:6px;font-size:15px;border:none;border-radius:6px;cursor:pointer}
#enable{background:#2ecc71;color:#fff}#disable{background:#e74c3c;color:#fff}
.param{margin:10px 0}
input[type=number]{width:80px;padding:4px}
#status{margin-top:20px;color:#95a5a6;font-size:14px}
</style></head><body>
<h2>双轮足 4关节位置控制</h2>
<div id="sliders"></div>
<div class="param">KP: <input type="number" id="kp" value="50" step="5" min="0" max="500">
    KD: <input type="number" id="kd" value="1.0" step="0.1" min="0" max="5"></div>
<div>
<button id="enable" onclick="sendCmd('enable',true)">使能</button>
<button id="disable" onclick="sendCmd('enable',false)">失能</button>
</div>
<div id="status">等待操作...</div>
<script>
const joints = %(joints)s;
const sliderRows = [];
function init(){
  const box = document.getElementById('sliders');
  joints.forEach((j,i)=>{
    const row=document.createElement('div');row.className='slider-row';
    const lab=document.createElement('label');lab.textContent=j[0];
    const sl=document.createElement('input');sl.type='range';sl.min=j[1];sl.max=j[2];sl.step=0.01;sl.value=0;
    const val=document.createElement('span');val.className='val';val.textContent='0.00 rad';
    sl.oninput=()=>{ val.textContent=parseFloat(sl.value).toFixed(2)+' rad'; scheduleSend(); };
    row.appendChild(lab);row.appendChild(sl);row.appendChild(val);
    box.appendChild(row); sliderRows.push(sl);
  });
}
let sendTimer=null;
function scheduleSend(){ clearTimeout(sendTimer); sendTimer=setTimeout(sendPos,80); }
function sendPos(){
  const pos=sliderRows.map(s=>parseFloat(s.value));
  const kp=parseFloat(document.getElementById('kp').value);
  const kd=parseFloat(document.getElementById('kd').value);
  fetch('/set_pos',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({pos, kp, kd})});
}
function sendCmd(type,val){
  fetch('/'+type,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({enable:val})});
  document.getElementById('status').textContent = type+' '+(val?'ON':'OFF');
}
init();
</script></body></html>
"""


class Handler(http.server.BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            body = HTML.replace('%(joints)s', json.dumps(JOINTS)).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        global current_pos, current_kp, current_kd
        length = int(self.headers.get('Content-Length', 0))
        try:
            data = json.loads(self.rfile.read(length).decode() or '{}')
        except Exception:
            data = {}
        if self.path == '/set_pos' and 'pos' in data:
            p = data['pos']
            for i in range(4):
                if i < len(p):
                    current_pos[i] = clamp(float(p[i]), JOINTS[i][1], JOINTS[i][2])
            current_kp = clamp(float(data.get('kp', DEFAULT_KP)), 0, 500)
            current_kd = clamp(float(data.get('kd', DEFAULT_KD)), 0, 5)
            publish_pos()
            self._ok({'ok': True})
        elif self.path == '/enable' or self.path == '/disable':
            enable = (self.path == '/enable')
            publish_enable(enable)
            self._ok({'enable': enable})
        else:
            self._ok({'ok': False})

    def _ok(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    global pub, enable_pub
    rclpy.init()
    node = PosCommandNode()
    pub = node.pub
    enable_pub = node.enable_pub
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    PORT = 8000
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("0.0.0.0", PORT), Handler) as httpd:
        print(f"[WEB] http://0.0.0.0:{PORT}  (控制4关节位置)")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
