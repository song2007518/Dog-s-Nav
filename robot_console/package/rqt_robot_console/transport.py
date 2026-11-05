"""One persistent WebSocket connection with serialized requests, never on the GUI thread."""
import itertools
import json
import queue
import threading
import time
from urllib.parse import urlparse

from PyQt5.QtCore import QThread, pyqtSignal

from .core import request, response_for


class RequestTimeout(TimeoutError):
    """No matching reply yet; the socket itself is still usable."""


class WsWorker(QThread):
    result = pyqtSignal(int, str, object)
    error = pyqtSignal(int, str, str)
    event = pyqtSignal(object)

    def __init__(self, url, parent=None):
        super().__init__(parent)
        parts = urlparse(url)
        if parts.scheme not in ('ws', 'wss') or not parts.hostname or parts.username or parts.password:
            raise ValueError('请输入合法的 ws:// 或 wss:// 地址，不包含用户名密码')
        self.url = url
        self.jobs = queue.PriorityQueue(maxsize=64)
        self.counter = itertools.count(1)
        self.stopping = threading.Event()
        self.lock = threading.Lock()
        self.active = None
        self.read_keys = set()
        self.reset_connection = threading.Event()
        self.reply_timeout=10.
        self.map_timeout=20.

    def submit(self, func, value=None, priority=10):
        if self.stopping.is_set():
            raise RuntimeError('连接正在关闭')
        with self.lock:
            if func.startswith('get_') and func in self.read_keys:
                return None
            seq = next(self.counter)
            self.jobs.put_nowait((priority, seq, func, value))
            if func.startswith('get_'):
                self.read_keys.add(func)
        return seq

    def cancel_pending(self):
        self.reset_connection.set()
        with self.lock:
            while True:
                try:
                    _, seq, func, _ = self.jobs.get_nowait()
                    self.read_keys.discard(func)
                    self.error.emit(seq, func, '请求已取消')
                except queue.Empty:
                    break
            sock = self.active
        if sock:
            try:
                sock.abort()
            except Exception:
                pass

    def shutdown(self):
        self.stopping.set()
        self.cancel_pending()

    def run(self):
        import websocket
        sock = None
        last_ping = time.monotonic()
        def event(raw):
            if not raw: raise ConnectionError('设备关闭了连接')
            if len(raw) > 80_000_000: raise ValueError('响应超出 80 MB 限制')
            msg = json.loads(raw)
            if not isinstance(msg, dict): return {}
            if msg.get('head', {}).get('type') == 'alg_error_code_notify':
                self.event.emit(msg)
            result = msg.get('data', {}).get('req_result', {})
            if isinstance(result, dict) and 'notify_stop_mapping_status' in str(result.get('req_func', '')):
                self.event.emit(msg)
            return msg
        try:
            while not self.stopping.is_set():
                job = None
                try:
                    try:
                        job = self.jobs.get(timeout=0 if sock else .1)
                    except queue.Empty:
                        if sock:
                            if time.monotonic()-last_ping > 5:
                                sock.ping(); last_ping=time.monotonic()
                            sock.settimeout(.1)
                            try: event(sock.recv())
                            except websocket.WebSocketTimeoutException: pass
                        continue
                    _, seq, func, value = job
                    if self.reset_connection.is_set():
                        if sock:
                            try:sock.close(timeout=.2)
                            except Exception:pass
                        sock=None
                        with self.lock:self.active=None
                        self.reset_connection.clear()
                    if sock is None:
                        sock = websocket.create_connection(self.url, timeout=4,
                                  http_no_proxy=['*'], enable_multithread=True,skip_utf8_validation=True)
                        last_ping=time.monotonic()
                        with self.lock: self.active=sock
                    if self.stopping.is_set(): break
                    # Never replay a mutation after losing an acknowledgement.
                    sock.send(json.dumps(request(func, value, seq), ensure_ascii=False))
                    deadline=time.monotonic()+(self.map_timeout if func in ('get_all_pgm_map','get_pgm_map') else self.reply_timeout)
                    while not self.stopping.is_set():
                        remaining=deadline-time.monotonic()
                        if remaining <= 0:
                            raise RequestTimeout('读取响应超时，将继续查询' if func.startswith('get_') else '等待匹配响应超时；操作结果未知，请查询状态后再决定是否重试')
                        sock.settimeout(min(remaining, 1))
                        try: msg=event(sock.recv())
                        except websocket.WebSocketTimeoutException: continue
                        response=response_for(msg, func, seq)
                        if response is not None:
                            if response.get('status') != 'ok':
                                # A rejected request is not a broken connection.
                                self.error.emit(seq,func,str(response.get('msg') or '设备拒绝了请求'))
                            else: self.result.emit(seq,func,response.get('data'))
                            break
                except RequestTimeout as exc:
                    # Do not disconnect a live App session just because one reply is late:
                    # this firmware stops navigation on disconnect. Late replies keep their
                    # original IDs and cannot acknowledge the next request.
                    if not self.stopping.is_set():self.error.emit(seq,func,str(exc))
                except Exception as exc:
                    if not self.stopping.is_set():
                        self.error.emit(0,'connection',str(exc))
                        if job:self.error.emit(job[1],job[2],str(exc))
                    if sock:
                        try: sock.close(timeout=.2)
                        except Exception: pass
                    sock=None
                    with self.lock: self.active=None
                    # Only queued reads / an explicit priority stop may proceed after disconnect.
                    kept=[]
                    with self.lock:
                        while True:
                            try: queued=self.jobs.get_nowait()
                            except queue.Empty: break
                            if queued[2].startswith('get_') or queued[2]=='stop_nav': kept.append(queued)
                            else: self.error.emit(queued[1],queued[2],'连接中断，未发送排队操作')
                        for queued in kept:self.jobs.put_nowait(queued)
                finally:
                    if job:
                        with self.lock:self.read_keys.discard(job[2])
        finally:
            with self.lock:self.active=None
            if sock:
                try:sock.close(timeout=.2)
                except Exception:pass


class RosMonitor(QThread):
    """Read-only ROS subscriptions. No publishers, services, SDK or motion commands."""
    telemetry = pyqtSignal(str, object)
    error = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.stopping = threading.Event()

    def shutdown(self):
        self.stopping.set()

    def run(self):
        context = node = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from nav_msgs.msg import Odometry, OccupancyGrid, Path
            from sensor_msgs.msg import LaserScan, BatteryState
            from std_msgs.msg import String, Header
            from tf2_ros import Buffer, TransformListener
            context = Context()
            rclpy.init(context=context, args=[])
            node = rclpy.create_node('rqt_robot_console_monitor', context=context)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            buffer = Buffer(node=node)
            listener = TransformListener(buffer, node, spin_thread=False)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.last_emit = {}
            odom_source = [None, 0.]
            def emit(kind, data):
                now = time.monotonic()
                if now-self.last_emit.get(kind, 0) < .1:
                    return
                self.last_emit[kind] = now
                self.telemetry.emit(kind, data)
            def pose_cb(msg):
                from .core import yaw_of,rotate_vector,multiply_quaternion
                age=(node.get_clock().now().nanoseconds-(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec))*1e-9
                if abs(age)>1.5:
                    emit('pose_unavailable','位姿源时间过期/时钟不同步：%.1f s'%age)
                    return
                if odom_source[0] not in ('localization_odom', 'slam_odom') or time.monotonic()-odom_source[1]>1.5:
                    emit('pose_unavailable', '尚无有效定位/建图位姿源，当前：'+str(odom_source[0]))
                    return
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                x, y = p.x, p.y
                yaw = yaw_of({'x':q.x,'y':q.y,'z':q.z,'w':q.w})
                frame = msg.header.frame_id
                if frame != 'map':
                    try:
                        transform = buffer.lookup_transform('map', frame, rclpy.time.Time.from_msg(msg.header.stamp))
                        t, r = transform.transform.translation, transform.transform.rotation
                        rotation={'x':r.x,'y':r.y,'z':r.z,'w':r.w}
                        rx,ry,_=rotate_vector(rotation,(x,y,p.z))
                        x,y=t.x+rx,t.y+ry
                        yaw=yaw_of(multiply_quaternion(rotation,{'x':q.x,'y':q.y,'z':q.z,'w':q.w}))
                        frame = 'map'
                    except Exception:
                        emit('pose_unavailable', '位姿 frame='+frame+'，等待到 map 的 TF')
                        return
                emit('pose', (x,y,yaw,frame))
            def scan_cb(msg):
                import math
                age=(node.get_clock().now().nanoseconds-(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec))*1e-9
                if abs(age)>1.0:
                    emit('scan_unavailable','激光源时间过期/时钟不同步：%.1f s'%age)
                    return
                pts = []
                for i,r in enumerate(msg.ranges):
                    if math.isfinite(r) and msg.range_min <= r <= min(msg.range_max,6):
                        a = msg.angle_min+i*msg.angle_increment
                        pts.append((r*math.cos(a),r*math.sin(a)))
                frame = msg.header.frame_id
                if frame != 'base_link':
                    try:
                        from .core import rotate_vector
                        tf = buffer.lookup_transform('base_link',frame,rclpy.time.Time.from_msg(msg.header.stamp)).transform
                        rotation={'x':tf.rotation.x,'y':tf.rotation.y,'z':tf.rotation.z,'w':tf.rotation.w}
                        rotated=[rotate_vector(rotation,(x,y,0)) for x,y in pts]
                        pts=[(tf.translation.x+x,tf.translation.y+y) for x,y,z in rotated]
                    except Exception:
                        emit('scan_unavailable','扫描 frame='+frame+'，等待到 base_link 的 TF')
                        return
                emit('scan', pts)
            node.create_subscription(Odometry, '/odom/current_pose', pose_cb, qos)
            node.create_subscription(LaserScan, '/laser_scan', scan_cb, qos)
            # Deployed robot uses Header.frame_id; older documentation says String.
            source_subscription = [None, None]
            def source_cb(m):
                value = m.frame_id if isinstance(m, Header) else m.data
                odom_source[:] = [value, time.monotonic()]
                emit('odom_type', value)
            def discover_source():
                types = dict(node.get_topic_names_and_types()).get('/tf_manager/odom_type', [])
                typ = Header if 'std_msgs/msg/Header' in types else String if 'std_msgs/msg/String' in types else None
                if typ is None or typ is source_subscription[1]: return
                if source_subscription[0]: node.destroy_subscription(source_subscription[0])
                odom_source[:] = [None, 0.]
                source_subscription[:] = [node.create_subscription(typ, '/tf_manager/odom_type', source_cb, qos), typ]
            node.create_timer(1., discover_source)
            discover_source()
            node.create_subscription(String, '/nav_status', lambda m: emit('custom_nav',m.data), qos)
            def path_cb(msg):
                import math
                age=(node.get_clock().now().nanoseconds-(msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec))*1e-9
                if msg.header.frame_id != 'map' or abs(age)>1.5:
                    emit('path_unavailable','路径坐标系不匹配或时间已过期')
                    return
                if len(msg.poses)>20000 or any(p.header.frame_id not in ('','map') for p in msg.poses):
                    emit('path_unavailable','路径数据或点坐标系无效')
                    return
                points=[(p.pose.position.x,p.pose.position.y) for p in msg.poses]
                if not all(math.isfinite(x) and math.isfinite(y) for x,y in points):
                    emit('path_unavailable','路径含无效坐标')
                    return
                emit('path',points)
            node.create_subscription(Path,'/custom_nav/path',path_cb,qos)
            node.create_subscription(BatteryState,'/battery_state',lambda m: emit('battery',m.percentage),qos)
            def map_cb(m):
                from .core import GridMap, yaw_of
                q=m.info.origin.orientation
                grid=GridMap('ros_live',m.info.width,m.info.height,m.info.resolution,
                     (m.info.origin.position.x,m.info.origin.position.y,yaw_of({'x':q.x,'y':q.y,'z':q.z,'w':q.w})),
                     list(m.data),m.header.frame_id,'ros')
                emit('map',grid)
            node.create_subscription(OccupancyGrid,'/map',map_cb,map_qos)
            self.telemetry.emit('ready','ROS 只读订阅已启动；等待话题')
            while not self.stopping.is_set():
                executor.spin_once(timeout_sec=.1)
        except Exception as exc:
            self.error.emit(str(exc))
        finally:
            if executor:
                executor.shutdown(timeout_sec=1)
            if node:
                node.destroy_node()
            if context and context.ok():
                context.shutdown()
