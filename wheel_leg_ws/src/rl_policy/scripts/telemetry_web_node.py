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
import base64
import hashlib
import json
import math
from pathlib import Path
import time
import threading
import http.server
import socket
import socketserver
from urllib.parse import urlsplit
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
        length = math.hypot(q.w, q.x, q.y, q.z)
        roll = pitch = yaw = None
        if msg.orientation_covariance[0] != -1 and math.isfinite(length) and length > 1e-9:
            w, x, y, z = (component / length for component in (q.w, q.x, q.y, q.z))
            roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
            pitch = math.asin(max(-1, min(1, 2 * (w*y - z*x))))
            yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
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
        with self.ws_lock:
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
        # WebSocket 文本帧: 0x81 + 长度 + 数据 (服务端掩码=无)
        frame = self._ws_frame(snapshot)
        with self.ws_lock:
            for client in self.ws_clients:
                client.publish(frame)

    @staticmethod
    def _ws_frame(data, opcode=1):
        """Encode a final server-to-client WebSocket frame."""
        header = bytearray([0x80 | opcode])
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


class TelemetryPeer:
    """Keep only the most recent snapshot while a client is writing a previous one."""

    def __init__(self, handler):
        self.handler = handler
        self.ready = threading.Condition()
        self.output_lock = threading.Lock()
        self.pending = None
        self.closed = False

    def publish(self, frame):
        with self.ready:
            self.pending = frame
            self.ready.notify()

    def stop(self):
        with self.ready:
            self.closed = True
            self.ready.notify()

    def write(self, frame):
        with self.output_lock:
            self.handler.wfile.write(frame)

    def send_loop(self):
        try:
            while True:
                with self.ready:
                    self.ready.wait_for(lambda: self.closed or self.pending is not None)
                    if self.closed:
                        return
                    frame, self.pending = self.pending, None
                with self.output_lock:
                    if not self.closed:
                        self.handler.wfile.write(frame)
        except (OSError, ValueError):
            try:
                self.handler.connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass


class TelemetryHandler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def do_GET(self):
        path = urlsplit(self.path).path
        if path in ("/", "/index.html"):
            page = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(page)))
            self.end_headers()
            self.wfile.write(page)
        elif path == "/ws":
            self.handle_websocket()
        else:
            self.send_error(404)

    def log_message(self, format, *args):
        pass

    def handle_websocket(self):
        key = self.headers.get("Sec-WebSocket-Key", "")
        if (not key or self.headers.get("Upgrade", "").lower() != "websocket"
                or self.headers.get("Sec-WebSocket-Version") != "13"):
            self.send_error(400)
            return
        accept = hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
        self.send_response(101)
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", base64.b64encode(accept).decode())
        self.end_headers()
        self.close_connection = True
        node = self.server.node
        peer = TelemetryPeer(self)
        with node.ws_lock:
            node.ws_clients.append(peer)
        threading.Thread(target=peer.send_loop, daemon=True).start()
        try:
            while True:
                header = self.rfile.read(2)
                if len(header) != 2:
                    break
                flags, length = header
                opcode = flags & 15
                if flags not in (0x88, 0x89, 0x8a) or not length & 0x80 or (length & 127) > 125:
                    # Subscribers receive telemetry. Application messages are unsupported.
                    code = 1003 if opcode in (1, 2) else 1002
                    peer.write(node._ws_frame(code.to_bytes(2, "big"), 8))
                    break
                mask = self.rfile.read(4)
                payload = self.rfile.read(length & 127)
                if len(mask) != 4 or len(payload) != (length & 127):
                    break
                payload = bytes(value ^ mask[i % 4] for i, value in enumerate(payload))
                if opcode == 8:
                    peer.stop()
                    peer.write(node._ws_frame(payload, 8))
                    break
                if opcode == 9:
                    peer.write(node._ws_frame(payload, 10))
        except OSError:
            pass
        finally:
            with node.ws_lock:
                node.ws_clients.remove(peer)
            peer.stop()


HTML_PAGE = Path(__file__).with_name("telemetry_dashboard.html").read_text(encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryWebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.server.shutdown()
    node.server.server_close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
