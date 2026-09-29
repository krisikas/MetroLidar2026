#!/usr/bin/env python3
"""
MetroLidar2026: Web Monitoring Dashboard Server.
Serves real-time obstacle detection telemetry, topic frequencies (Hz), 
and 3D LiDAR scene stream over HTTP/SSE.
Works 100% offline with zero external pip dependencies.
"""
import json
import math
import os
import struct
import sys
import threading
import time
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import Float32, Header
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from metro_obstacle_detector_interfaces.msg import ObstacleArray, SystemHealth, TrackProfile

class RateMeter:
    """Thread-safe exponential moving average frequency estimator (Hz)."""
    def __init__(self, alpha=0.25, timeout=2.0):
        self.lock = threading.RLock()
        self.alpha = alpha
        self.timeout = timeout
        self.last_time = 0.0
        self.hz = 0.0

    def tick(self):
        with self.lock:
            now = time.time()
            if self.last_time > 0.0:
                dt = now - self.last_time
                if dt > 0.001:
                    inst_hz = 1.0 / dt
                    if self.hz == 0.0:
                        self.hz = inst_hz
                    else:
                        self.hz = (1.0 - self.alpha) * self.hz + self.alpha * inst_hz
            self.last_time = now

    def get_hz(self):
        with self.lock:
            if self.last_time == 0.0 or (time.time() - self.last_time > self.timeout):
                return 0.0
            return round(self.hz, 1)

class DashboardState:
    """Thread-safe store for latest telemetry, track, and obstacle data."""
    def __init__(self):
        self.lock = threading.RLock()
        self.train_speed = 0.0          # m/s
        self.train_speed_kmh = 0.0      # km/h
        self.min_distance = -1.0        # m
        self.ttc = -1.0                 # s
        self.braking_distance = 0.0     # m
        self.alert_status = 0           # 0: CLEAR, 1: ADVISORY, 2: WARNING, 3: EMERGENCY_BRAKE
        self.emergency_brake = False
        self.operational_mode = "Автономный режим"
        self.obstacles = []
        self.path_points = []
        self.cloud_points = []
        self.telemetry = {
            'latency_ms': 0.0,
            'input_pts': 0,
            'valid_pts': 0,
            'active_tracks': 0,
            'gpu': False
        }
        self.rate_lidar = RateMeter()
        self.rate_obstacles = RateMeter()
        self.rate_path = RateMeter()
        self.rate_telemetry = RateMeter()
        self.timestamp = time.time()
        self.listeners = []

    def update_obstacles(self, msg: ObstacleArray):
        self.rate_obstacles.tick()
        with self.lock:
            self.train_speed = float(msg.train_speed)
            self.train_speed_kmh = round(self.train_speed * 3.6, 1)
            self.min_distance = round(float(msg.min_distance_to_obstacle), 2)
            self.ttc = round(float(msg.time_to_collision), 2)
            self.braking_distance = round(float(msg.emergency_braking_distance), 2)
            self.alert_status = int(msg.alert_status)
            self.emergency_brake = bool(msg.emergency_brake_required)

            obs_list = []
            for obs in msg.obstacles:
                obs_list.append({
                    'id': obs.id,
                    'category': obs.obstacle_type,
                    'threat': obs.threat_level,
                    'distance': round(float(obs.distance), 2),
                    'x': round(float(obs.position.x), 2),
                    'y': round(float(obs.position.y), 2),
                    'z': round(float(obs.position.z), 2),
                    'size_x': round(float(obs.size.x), 2),
                    'size_y': round(float(obs.size.y), 2),
                    'size_z': round(float(obs.size.z), 2),
                    'penetration': round(float(obs.gauge_penetration), 3),
                    'pts': obs.point_count,
                    'hits': obs.tracking_frames,
                    'confirmed': True
                })
            self.obstacles = obs_list
            self.timestamp = time.time()

        self._notify_listeners()

    def update_lidar_tick(self, msg: PointCloud2):
        self.rate_lidar.tick()
        try:
            n_pts = msg.width * msg.height
            if n_pts > 0 and msg.data:
                step = msg.point_step
                data = msg.data
                ox, oy, oz = 0, 4, 8
                for f in msg.fields:
                    if f.name == 'x': ox = f.offset
                    elif f.name == 'y': oy = f.offset
                # Выборка ~14000 точек для плотного, детального и четкого отображения тоннеля и объектов
                stride = max(1, n_pts // 14000)
                pts = []
                data_len = len(data)
                for i in range(0, n_pts, stride):
                    off = i * step
                    if off + max(ox, oy, oz) + 4 <= data_len:
                        x = struct.unpack_from('<f', data, off + ox)[0]
                        y = struct.unpack_from('<f', data, off + oy)[0]
                        z = struct.unpack_from('<f', data, off + oz)[0]
                        if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                            if -155.0 <= y <= 0.5 and abs(x) <= 6.0 and -3.5 <= z <= 3.5:
                                pts.append([round(x, 2), round(y, 2), round(z, 2)])
                with self.lock:
                    self.cloud_points = pts
                    if n_pts > 0:
                        self.telemetry['input_pts'] = n_pts
                now = time.time()
                if now - getattr(self, '_last_lidar_notify', 0.0) >= 0.06:
                    self._last_lidar_notify = now
                    self._notify_listeners()
        except Exception:
            pass

    def update_speed(self, msg: Float32):
        with self.lock:
            self.train_speed = float(msg.data)
            self.train_speed_kmh = round(self.train_speed * 3.6, 1)
        self._notify_listeners()

    def update_path(self, msg: Path):
        self.rate_path.tick()
        with self.lock:
            pts = []
            for i, p in enumerate(msg.poses):
                if i % 2 == 0:
                    pts.append([
                        round(float(p.pose.position.x), 2),
                        round(float(p.pose.position.y), 2),
                        round(float(p.pose.position.z), 2)
                    ])
            self.path_points = pts
        self._notify_listeners()

    def update_telemetry(self, msg: SystemHealth):
        self.rate_telemetry.tick()
        with self.lock:
            self.telemetry = {
                'latency_ms': round(float(msg.processing_time_ms), 1),
                'input_pts': int(msg.input_points),
                'valid_pts': int(msg.valid_points),
                'active_tracks': int(msg.active_tracks),
                'gpu': bool(msg.gpu_accelerated)
            }
        self._notify_listeners()

    def to_dict(self):
        with self.lock:
            now = time.time()
            dds_active = (now - self.timestamp) < 3.0 or self.rate_obstacles.get_hz() > 0.0 or self.rate_lidar.get_hz() > 0.0
            return {
                'train_speed': self.train_speed,
                'train_speed_kmh': self.train_speed_kmh,
                'min_distance': self.min_distance,
                'ttc': self.ttc,
                'braking_distance': self.braking_distance,
                'alert_status': self.alert_status,
                'emergency_brake': self.emergency_brake,
                'operational_mode': self.operational_mode,
                'obstacles': self.obstacles,
                'path_points': self.path_points,
                'cloud_points': self.cloud_points,
                'telemetry': self.telemetry,
                'topic_rates': {
                    'lidar_hz': self.rate_lidar.get_hz(),
                    'obstacles_hz': self.rate_obstacles.get_hz(),
                    'path_hz': self.rate_path.get_hz(),
                    'telemetry_hz': self.rate_telemetry.get_hz()
                },
                'dds_active': dds_active,
                'timestamp': self.timestamp
            }

    def register_listener(self, queue):
        with self.lock:
            self.listeners.append(queue)

    def unregister_listener(self, queue):
        with self.lock:
            if queue in self.listeners:
                self.listeners.remove(queue)

    def _notify_listeners(self):
        with self.lock:
            if not self.listeners:
                return
            snapshot = self.to_dict()
            listeners_copy = list(self.listeners)
        for q in listeners_copy:
            try:
                q.put_nowait(snapshot)
            except Exception:
                pass

GLOBAL_STATE = DashboardState()

class ThreadedDashboardServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

class DashboardRequestHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, web_dir=None, **kwargs):
        self.web_dir = web_dir
        super().__init__(*args, directory=web_dir, **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate, max-age=0')
        self.send_header('Pragma', 'no-cache')
        self.send_header('Expires', '0')
        super().end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/api/state':
            data = json.dumps(GLOBAL_STATE.to_dict()).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(data)
            return

        if parsed.path == '/events':
            # Server-Sent Events (SSE) Stream
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-cache, no-transform')
            self.send_header('Connection', 'keep-alive')
            self.send_header('X-Accel-Buffering', 'no')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()

            import queue
            q = queue.Queue(maxsize=30)
            GLOBAL_STATE.register_listener(q)

            try:
                # Send initial state immediately
                init_data = json.dumps(GLOBAL_STATE.to_dict())
                self.wfile.write(f"data: {init_data}\n\n".encode('utf-8'))
                self.wfile.flush()

                while True:
                    try:
                        state_dict = q.get(timeout=0.5)
                        payload = json.dumps(state_dict)
                        self.wfile.write(f"data: {payload}\n\n".encode('utf-8'))
                        self.wfile.flush()
                    except queue.Empty:
                        # Send periodic snapshot so Hz counters stay updated
                        payload = json.dumps(GLOBAL_STATE.to_dict())
                        self.wfile.write(f"data: {payload}\n\n".encode('utf-8'))
                        self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError, Exception):
                pass
            finally:
                GLOBAL_STATE.unregister_listener(q)
                self.close_connection = True
            return

        # Serve static assets
        return super().do_GET()

    def do_POST(self):
        self.send_response(404)
        self.end_headers()

class MetroWebGuiNode(Node):
    def __init__(self):
        super().__init__('metro_web_gui_node')

        self.declare_parameter('port', 8080)
        self.declare_parameter('web_dir', '')
        self.declare_parameter('lidar_topic', '/lidar_points')

        port = self.get_parameter('port').value
        web_dir_param = self.get_parameter('web_dir').value
        lidar_topic = self.get_parameter('lidar_topic').value

        # Locate web directory
        if web_dir_param and os.path.exists(web_dir_param):
            self.web_dir = web_dir_param
        else:
            try:
                from ament_index_python.packages import get_package_share_directory
                share_web = os.path.join(get_package_share_directory('metro_web_gui'), 'web')
                if os.path.exists(share_web):
                    self.web_dir = share_web
                else:
                    raise FileNotFoundError
            except Exception:
                this_dir = os.path.dirname(os.path.abspath(__file__))
                cand1 = os.path.abspath(os.path.join(this_dir, '..', 'web'))
                cand2 = os.path.abspath(os.path.join(this_dir, 'web'))
                if os.path.exists(cand1):
                    self.web_dir = cand1
                elif os.path.exists(cand2):
                    self.web_dir = cand2
                else:
                    self.web_dir = os.getcwd()

        # QoS Profiles matching C++ ObstacleDetectorNode exactly
        qos_cloud = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        qos_rel_latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_rel_volatile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.sub_cloud = self.create_subscription(
            PointCloud2, lidar_topic, GLOBAL_STATE.update_lidar_tick, qos_cloud)
        self.sub_obstacles = self.create_subscription(
            ObstacleArray, '/metro/obstacles', GLOBAL_STATE.update_obstacles, qos_rel_latched)
        self.sub_path = self.create_subscription(
            Path, '/metro/track_path', GLOBAL_STATE.update_path, qos_rel_latched)
        self.sub_speed = self.create_subscription(
            Float32, '/metro/train_speed', GLOBAL_STATE.update_speed, qos_rel_volatile)
        self.sub_telemetry = self.create_subscription(
            SystemHealth, '/metro/telemetry', GLOBAL_STATE.update_telemetry, qos_rel_volatile)

        # Start HTTP / SSE Server in background thread
        handler = lambda *args, **kwargs: DashboardRequestHandler(*args, web_dir=self.web_dir, **kwargs)
        self.server = ThreadedDashboardServer(('0.0.0.0', port), handler)
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f"============================================================")
        self.get_logger().info(f"MetroLidar Web Dashboard Server Started")
        self.get_logger().info(f"Serving at: http://0.0.0.0:{port}")
        self.get_logger().info(f"LiDAR Monitoring: {lidar_topic}")
        self.get_logger().info(f"Web assets: {self.web_dir}")
        self.get_logger().info(f"============================================================")

def main(args=None):
    rclpy.init(args=args)
    node = MetroWebGuiNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
