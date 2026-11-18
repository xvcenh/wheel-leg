#!/usr/bin/env python3
"""
WheelLeg 遥测仪表盘 — WebSocket 实时数据可视化

用法:
  ros2 run rl_policy telemetry_web_node.py
  # 浏览器访问 http://<RDK_IP>:8080

订阅:
  /mcu/imu, /mcu/joint_states, /mcu/motor_temp, /mcu/power, /mcu/wheel_states

只读显示收到的 ROS 遥测数据。
"""
import json
import math
from pathlib import Path
import time
import threading
import http.server
import socketserver
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray


def finite_values(value):
    if isinstance(value, list):
        return [finite_values(item) for item in value]
    return value if value is not None and math.isfinite(value) else None


class TelemetryWebNode(Node):
    def __init__(self):
        super().__init__('telemetry_web_node')
        self.declare_parameter('port', 8080)
        port = self.get_parameter('port').value

        # 数据缓存
        self.data_lock = threading.Lock()
        self.data = {
            "imu": {"roll": None, "pitch": None, "yaw": None, "gyro": [], "accel": []},
            "joints": {"pos": [], "vel": [], "tor": []},
            "wheels": {"pos": [], "vel": [], "tor": []},
            "temp": {"tmos": [], "tcoil": []},
            "power": {"voltage": None, "imu_temp": None, "protect": None},
        }
        self.received = {}
        self.ws_clients = []
        self.ws_lock = threading.Lock()

        # 订阅
        self.create_subscription(Imu, '/mcu/imu', self.cb_imu, 10)
        self.create_subscription(JointState, '/mcu/joint_states', self.cb_joints, 10)
        self.create_subscription(Float32MultiArray, '/mcu/motor_temp', self.cb_temp, 10)
        self.create_subscription(Float32MultiArray, '/mcu/power', self.cb_power, 10)
        self.create_subscription(Float32MultiArray, '/mcu/wheel_states', self.cb_wheels, 10)

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
            self.received['imu'] = time.monotonic()
            self.data['imu'] = {
                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                'gyro': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'accel': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
            }

    def cb_joints(self, msg):
        with self.data_lock:
            self.received['joints'] = time.monotonic()
            self.data['joints'] = {
                'pos': list(msg.position),
                'vel': list(msg.velocity),
                'tor': list(msg.effort),
            }

    def cb_wheels(self, msg):
        d = list(msg.data)
        with self.data_lock:
            self.received['wheels'] = time.monotonic()
            self.data['wheels'] = {
                'pos': d[:2],
                'vel': d[2:4],
                'tor': d[4:6],
            }

    def cb_temp(self, msg):
        d = list(msg.data)
        with self.data_lock:
            self.received['temp'] = time.monotonic()
            self.data['temp'] = {
                'tmos': d[:4],
                'tcoil': d[4:8],
            }

    def cb_power(self, msg):
        d = list(msg.data)
        with self.data_lock:
            self.received['power'] = time.monotonic()
            self.data['power'] = {
                'voltage': d[0] if len(d)>0 else None,
                'imu_temp': d[1] if len(d)>1 else None,
                'protect': d[2] if len(d)>2 else None,
            }

    def push_data(self):
        if not self.ws_clients:
            return
        with self.data_lock:
            now = time.monotonic()
            payload = {
                topic: {key: finite_values(value) for key, value in sample.items()}
                for topic, sample in self.data.items()
            }
            payload["age_s"] = {topic: round(now - ts, 3) for topic, ts in self.received.items()}
            snapshot = json.dumps(payload, allow_nan=False).encode("utf-8")
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


HTML_PAGE = Path(__file__).with_name("telemetry_dashboard.html").read_text(encoding="utf-8")


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
