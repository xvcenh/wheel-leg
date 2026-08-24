#!/usr/bin/env python3
"""
WheelLeg 遥测仪表盘 — WebSocket 实时数据可视化

用法:
  ros2 run rl_policy telemetry_web_node
  # 浏览器访问 http://<RDK_IP>:8080

订阅:
  /mcu/imu, /mcu/joint_states, /mcu/motor_temp, /mcu/power, /mcu/wheel_states

发布:
  /mcu/enable (Bool), /mcu/pos_cmd (Float32MultiArray)
"""
import json
import math
import threading
import http.server
import socketserver
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray, Bool


class TelemetryWebNode(Node):
    def __init__(self):
        super().__init__('telemetry_web_node')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        # 数据缓存
        self.data_lock = threading.Lock()
        self.data = {
            'imu': {'roll': 0, 'pitch': 0, 'yaw': 0, 'gyro': [0,0,0], 'accel': [0,0,0]},
            'joints': {'pos': [0,0,0,0], 'vel': [0,0,0,0], 'tor': [0,0,0,0]},
            'wheels': {'pos': [0,0], 'vel': [0,0], 'tor': [0,0]},
            'temp': {'tmos': [0,0,0,0], 'tcoil': [0,0,0,0]},
            'power': {'voltage': 0, 'imu_temp': 0, 'protect': 0},
        }
        self.ws_clients = []
        self.ws_lock = threading.Lock()

        # 订阅
        self.create_subscription(Imu, '/mcu/imu', self.cb_imu, 10)
        self.create_subscription(JointState, '/mcu/joint_states', self.cb_joints, 10)
        self.create_subscription(Float32MultiArray, '/mcu/motor_temp', self.cb_temp, 10)
        self.create_subscription(Float32MultiArray, '/mcu/power', self.cb_power, 10)
        self.create_subscription(Float32MultiArray, '/mcu/wheel_states', self.cb_wheels, 10)

        # 发布（使能控制）
        self.pub_enable = self.create_publisher(Bool, '/mcu/enable', 10)

        # 启动 HTTP + WebSocket 服务器
        self.server = TelemetryHTTPServer(('', port), TelemetryHandler)
        self.server.node = self
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()
        self.get_logger().info(f'Web dashboard on http://0.0.0.0:{port}')

        # 定时推送
        self.create_timer(0.1, self.push_data)

    def cb_imu(self, msg):
        q = msg.orientation
        # 四元数→欧拉角
        sinr = 2*(q.w*q.x + q.y*q.z)
        cosr = 1 - 2*(q.x*q.x + q.y*q.y)
        roll = math.atan2(sinr, cosr)
        sinp = 2*(q.w*q.y - q.z*q.x)
        pitch = math.asin(max(-1, min(1, sinp)))
        siny = 2*(q.w*q.z + q.x*q.y)
        cosy = 1 - 2*(q.y*q.y + q.z*q.z)
        yaw = math.atan2(siny, cosy)
        with self.data_lock:
            self.data['imu'] = {
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'gyro': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            }

    def cb_joints(self, msg):
        with self.data_lock:
            self.data['joints'] = {
                'pos': list(msg.position),
                'vel': list(msg.velocity),
                'tor': list(msg.effort),
            }

    def cb_wheels(self, msg):
        d = list(msg.data)
        n = len(d) // 3
        with self.data_lock:
            self.data['wheels'] = {
                'pos': d[:n] if n>=2 else [0,0],
                'vel': d[n:2*n] if n>=2 else [0,0],
                'tor': d[2*n:3*n] if n>=2 else [0,0],
            }

    def cb_temp(self, msg):
        d = list(msg.data)
        with self.data_lock:
            self.data['temp'] = {
                'tmos': d[:4] if len(d)>=4 else d,
                'tcoil': d[4:8] if len(d)>=8 else [0,0,0,0],
            }

    def cb_power(self, msg):
        d = list(msg.data)
        with self.data_lock:
            self.data['power'] = {
                'voltage': d[0] if len(d)>0 else 0,
                'imu_temp': d[1] if len(d)>1 else 0,
                'protect': int(d[2]) if len(d)>2 else 0,
            }

    def push_data(self):
        if not self.ws_clients:
            return
        with self.data_lock:
            snapshot = json.dumps(self.data).encode('utf-8')
        # WebSocket 文本帧: 0x82 + 长度 + 数据 (服务端掩码=无)
        frame = self._ws_frame(snapshot)
        with self.ws_lock:
            for client in self.ws_clients[:]:
                try:
                    client.write(frame)
                    client.flush()
                except Exception:
                    # 客户端已断开（handle_websocket 线程可能已移除），防御性移除
                    try:
                        self.ws_clients.remove(client)
                    except ValueError:
                        pass

    @staticmethod
    def _ws_frame(data):
        """编码 WebSocket 文本帧 (服务端→客户端, 无掩码)"""
        header = bytearray([0x81])  # FIN=1, opcode=1(text)
        length = len(data)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.append(126)
            header.extend(length.to_bytes(2, 'big'))
        else:
            header.append(127)
            header.extend(length.to_bytes(8, 'big'))
        return bytes(header) + data


class TelemetryHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


class TelemetryHandler(http.server.BaseHTTPRequestHandler):
    node = None  # set by TelemetryHTTPServer

    def do_GET(self):
        if self.path == '/' or self.path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode('utf-8'))
        elif self.path == '/ws':
            self.handle_websocket()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # 静默

    def handle_websocket(self):
        import base64, hashlib, os, struct
        # WebSocket 握手
        key = self.headers.get('Sec-WebSocket-Key')
        if not key:
            self.send_response(400)
            self.end_headers()
            return
        accept = base64.b64encode(
            hashlib.sha1((key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()).digest()
        ).decode()
        self.send_response(101)
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', accept)
        self.end_headers()
        # 注册客户端
        with self.server.node.ws_lock:
            self.server.node.ws_clients.append(self.wfile)
        # 保持连接
        try:
            while True:
                data = self.rfile.read(1)
                if not data:
                    break
        except Exception:
            pass
        finally:
            with self.server.node.ws_lock:
                if self.wfile in self.server.node.ws_clients:
                    self.server.node.ws_clients.remove(self.wfile)


HTML_PAGE = '''<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>WheelLeg 遥测仪表盘</title>
<style>
body{margin:0;padding:20px;background:#1a1a2e;color:#e0e0e0;font-family:monospace}
h1{text-align:center;color:#0ff}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:20px;max-width:1200px;margin:0 auto}
.card{background:#16213e;border-radius:10px;padding:15px}
.card h2{margin:0 0 10px;color:#0ff;font-size:16px}
.bar-row{display:flex;align-items:center;gap:10px;margin:4px 0}
.bar-label{width:80px;text-align:right;font-size:12px}
.bar-track{flex:1;height:20px;background:#0d1b2a;border-radius:4px;position:relative}
.bar-fill{height:100%;background:#0f8;border-radius:4px;transition:width 0.1s}
.bar-val{width:70px;font-size:12px}
.green{color:#0f8}.yellow{color:#ff0}.red{color:#f44}
</style></head><body>
<h1>WheelLeg 遥测仪表盘</h1>
<div class="grid">
  <div class="card"><h2>IMU 姿态</h2>
    <div class="bar-row"><span class="bar-label">Roll</span><div class="bar-track"><div class="bar-fill" id="roll" style="width:50%"></div></div><span class="bar-val" id="roll_v">0.0</span></div>
    <div class="bar-row"><span class="bar-label">Pitch</span><div class="bar-track"><div class="bar-fill" id="pitch" style="width:50%"></div></div><span class="bar-val" id="pitch_v">0.0</span></div>
    <div class="bar-row"><span class="bar-label">Yaw</span><div class="bar-track"><div class="bar-fill" id="yaw" style="width:50%"></div></div><span class="bar-val" id="yaw_v">0.0</span></div>
  </div>
  <div class="card"><h2>电池电压</h2>
    <div class="bar-row"><span class="bar-label">V (V)</span><div class="bar-track"><div class="bar-fill" id="vbat" style="width:90%"></div></div><span class="bar-val" id="vbat_v">0.0</span></div>
    <div class="bar-row"><span class="bar-label">状态</span><span class="bar-val" id="pstate">--</span></div>
  </div>
  <div class="card"><h2>关节电机</h2>
    <div id="joints"></div>
  </div>
  <div class="card"><h2>轮毂电机</h2>
    <div id="wheels"></div>
  </div>
  <div class="card"><h2>MOS 温度</h2><div id="tmos"></div></div>
  <div class="card"><h2>线圈温度</h2><div id="tcoil"></div></div>
</div>
<script>
const ws = new WebSocket('ws://'+location.host+'/ws');
const jointNames = ['hip_R','knee_R','hip_L','knee_L'];
const wheelNames = ['左轮','右轮'];
function bar(val,min,max,id,prefix){
  const pct = Math.max(0,Math.min(100,(val-min)/(max-min)*100));
  const el=document.getElementById(id); if(el) el.style.width=pct+'%';
  const v=document.getElementById(id+'_v'); if(v) v.textContent=val.toFixed(2);
}
ws.onmessage = function(e){
  const d = JSON.parse(e.data);
  bar(d.imu.roll,-3.14,3.14,'roll','rad');
  bar(d.imu.pitch,-1.57,1.57,'pitch','rad');
  bar(d.imu.yaw,-3.14,3.14,'yaw','rad');
  bar(d.power.voltage,18,25,'vbat','V');
  const ps=document.getElementById('pstate');
  if(d.power.protect==0){ps.textContent='正常';ps.className='green';}
  else if(d.power.protect==1){ps.textContent='警告';ps.className='yellow';}
  else{ps.textContent='锁定';ps.className='red';}
  // joints
  let jh='';
  for(let i=0;i<4;i++){
    const p=d.joints.pos[i]||0,v=d.joints.vel[i]||0,t=d.joints.tor[i]||0;
    jh+=`<div class="bar-row"><span class="bar-label">${jointNames[i]}</span><div class="bar-track"><div class="bar-fill" style="width:${Math.max(0,Math.min(100,(p+3.14)/6.28*100))}%"></div></div><span class="bar-val">${p.toFixed(2)}</span></div>`;
  }
  document.getElementById('joints').innerHTML=jh;
  // wheels — 显示位置(累计转数) + 速度
  let wh='';
  for(let i=0;i<2;i++){
    const p=d.wheels.pos[i]||0,v=d.wheels.vel[i]||0;
    wh+=`<div class="bar-row"><span class="bar-label">${wheelNames[i]}位</span><div class="bar-track"><div class="bar-fill" style="width:${Math.max(0,Math.min(100,Math.abs(p)/50*100))}%;background:#08f"></div></div><span class="bar-val">${p.toFixed(1)} rad</span></div>`;
    wh+=`<div class="bar-row"><span class="bar-label">${wheelNames[i]}速</span><div class="bar-track"><div class="bar-fill" style="width:${Math.max(0,Math.min(100,Math.abs(v)/45*100))}%;background:#0f8"></div></div><span class="bar-val">${v.toFixed(1)} rad/s</span></div>`;
  }
  document.getElementById('wheels').innerHTML=wh;
  // temps
  let th='';
  for(let i=0;i<4;i++){th+=`<div class="bar-row"><span class="bar-label">J${i}</span><div class="bar-track"><div class="bar-fill" style="width:${Math.min(100,d.temp.tmos[i]/80*100)}%;background:${d.temp.tmos[i]>60?'#f44':'#0f8'}"></div></div><span class="bar-val">${d.temp.tmos[i].toFixed(0)}°C</span></div>`;}
  document.getElementById('tmos').innerHTML=th;
  let ch='';
  for(let i=0;i<4;i++){ch+=`<div class="bar-row"><span class="bar-label">J${i}</span><div class="bar-track"><div class="bar-fill" style="width:${Math.min(100,d.temp.tcoil[i]/100*100)}%;background:${d.temp.tcoil[i]>70?'#f44':'#08f'}"></div></div><span class="bar-val">${d.temp.tcoil[i].toFixed(0)}°C</span></div>`;}
  document.getElementById('tcoil').innerHTML=ch;
};
ws.onclose = function(){document.body.innerHTML='<h1>连接断开，刷新重试</h1>';};
</script></body></html>'''


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.server.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
