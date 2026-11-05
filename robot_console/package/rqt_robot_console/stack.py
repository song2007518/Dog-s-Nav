"""Local start_all supervisor. Reading status never launches ROS or contacts the dog.

The vendor workspace script retains responsibility for posture and cleanup. A
running process, an exited process and a healthy robot are deliberately distinct.
"""
import fcntl
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time
from contextlib import contextmanager

from PyQt5.QtCore import QThread, pyqtSignal


DEFAULT_LOG_DIR = Path('/tmp/custom_nav_launch')


def default_start_all():
    override = os.environ.get('ROBOT_CONSOLE_START_ALL')
    if override:
        return Path(override).expanduser().resolve()
    # Works from both package/ and the local colcon install/ tree.
    for root in Path(__file__).resolve().parents:
        candidate = root / 'scripts/start_all.sh'
        if candidate.is_file() and (root / 'src/custom_nav/package.xml').is_file():
            return candidate
    return Path.home() / 'robot_ws/robot_sdk_ws/scripts/start_all.sh'


def process_info(pid):
    try:
        folder = Path('/proc') / str(int(pid))
        stat = (folder / 'stat').read_text().rsplit(')', 1)[1].split()
        if folder.stat().st_uid != os.getuid() or stat[0] in ('Z', 'X'):
            return None
        args = (folder / 'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
        return {'pid': int(pid), 'ppid': int(stat[1]), 'start': stat[19], 'args': args,
                'cwd': os.readlink(folder / 'cwd')}
    except (OSError, ValueError, IndexError):
        return None


def runs_script(info, script):
    if not info or len(info['args']) < 2 or Path(info['args'][0]).name not in ('bash', 'sh'):
        return False
    # Match the script operand, never a shell command string or a grep argument.
    args = info['args'][1:]
    if args[0] == '--':
        args = args[1:]
    if not args or args[0].startswith('-') or '--stop' in args[1:]:
        return False
    path = Path(args[0])
    return (path if path.is_absolute() else Path(info['cwd']) / path).resolve() == script.resolve()


def stack_environment():
    env = os.environ.copy()
    for key in ('ROS_LOCALHOST_ONLY', 'ZENOH_SESSION_CONFIG_URI', 'ZENOH_ROUTER_CONFIG_URI',
                'XDG_CONFIG_HOME', 'RBN_DETACHED', 'ROBOT_CONSOLE_CONNECT_READONLY'):
        env.pop(key, None)
    env.update(ROS_DOMAIN_ID='24', RMW_IMPLEMENTATION='rmw_zenoh_cpp', RBN_DETACHED='1',
               PYTHONUNBUFFERED='1')
    return env


class StackManager:
    def __init__(self, script=None, log_dir=DEFAULT_LOG_DIR):
        self.script = Path(script if script is not None else default_start_all()).resolve()
        self.log_dir = Path(log_dir)
        self.child = None
        self.last_exit = None
        self._tail_key = None
        self._tail_text = ''

    @property
    def log_path(self):
        return self.log_dir / 'boot.log'

    @contextmanager
    def locked(self):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        with (self.log_dir / 'rqt.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            yield

    def metadata(self):
        try:
            value = json.loads((self.log_dir / 'rqt_process.json').read_text())
            return value if isinstance(value, dict) else {}
        except (OSError, ValueError):
            return {}

    def save_metadata(self, value):
        target = self.log_dir / 'rqt_process.json'
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(value, ensure_ascii=False))
        temp.replace(target)

    def status(self):
        if self.child and self.child.poll() is not None:
            self.last_exit = self.child.returncode
            self.child = None
        processes, components = [], []
        for folder in Path('/proc').iterdir():
            if not folder.name.isdigit():
                continue
            info = process_info(folder.name)
            if runs_script(info, self.script):
                processes.append(info)
            elif info:
                args = info['args']
                names = [Path(a).name for a in args[:3]]
                if 'custom_nav_node' in names or 'ws_map_pub.py' in names:
                    components.append(info['pid'])
        meta = self.metadata()
        # bash command substitutions briefly inherit the outer shell's cmdline.
        # They are children of the launcher, not additional launcher instances.
        candidate_pids = {p['pid'] for p in processes}
        processes = [p for p in processes if p['ppid'] not in candidate_pids]
        stopping = any(p['pid'] == meta.get('pid') and p['start'] == meta.get('start')
                       and meta.get('stop_at') for p in processes)
        conflict = ''
        try:
            pid = int((self.log_dir / 'boot.pid').read_text().strip())
            if process_info(pid) and not any(p['pid'] == pid for p in processes):
                conflict = 'PID 文件指向其他进程，已禁止启动和停止该 PID'
        except (OSError, ValueError):
            pass
        if len(processes) > 1:
            conflict = '发现多个 start_all 实例，请先在原终端处理'
        if not processes and components:
            conflict = '发现独立或残留的 custom_nav / ws_map_pub，请先在原终端停止'
        if conflict:
            state, text = 'blocked', conflict
        elif processes:
            state = 'stopping' if stopping else 'running'
            text = '收工流程执行中，等待脚本退出' if stopping else '启动脚本运行中 · 设备状态需另行核验'
            if stopping and time.time() - meta.get('stop_at', time.time()) > 60:
                text = '收工超过 60 秒，检查日志和狗的姿态；未强杀进程'
        else:
            state = 'idle'
            text = '未启动'
            if meta or self.last_exit is not None:
                text = '启动脚本已退出 · 未核验狗的姿态'
            if self.last_exit not in (None, 0):
                text = '启动脚本异常退出（%s）· 请检查日志' % self.last_exit
        return {'state': state, 'text': text, 'processes': processes,
                'components': components, 'busy': bool(processes or components or conflict),
                'can_stop': len(processes) == 1 and not stopping,
                'exit_code': self.last_exit}

    def local_checks(self):
        root = self.script.parent.parent
        required = [self.script, Path('/opt/ros/humble/setup.bash'), root / 'install/setup.bash',
                    root / 'scripts/ws_cmd.py', root / 'scripts/ws_map_pub.py', root / 'scripts/cam_view.py',
                    root / 'install/custom_nav/lib/custom_nav/custom_nav_node',
                    Path('/opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd'),
                    Path.home() / '.config/zenoh/pc_router.json5',
                    Path.home() / '.config/robot_nav/rviz_readonly.rviz']
        missing = [str(p) for p in required if not p.is_file()]
        missing += [name for name in ('bash', 'sshpass', 'ssh', 'ping', 'timeout', 'rviz2')
                    if shutil.which(name) is None]
        return missing

    def start(self):
        with self.locked():
            status = self.status()
            if status['busy']:
                raise RuntimeError(status['text'])
            if not self.script.is_file():
                raise RuntimeError('找不到 start_all.sh：' + str(self.script))
            # Run the same --boot branch without a second fork. This gives us the
            # actual shell PID immediately; the script still writes its boot.pid.
            with self.log_path.open('wb') as output:
                self.child = subprocess.Popen(['/bin/bash', str(self.script), '--boot'],
                    cwd=str(self.script.parent.parent), env=stack_environment(),
                    stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                    start_new_session=True, close_fds=True)
            info = process_info(self.child.pid)
            self.save_metadata({'pid': self.child.pid, 'start': info['start'] if info else '',
                                'started_at': time.time(), 'script': str(self.script)})
            self.last_exit = None
            return self.child.pid

    def stop(self):
        with self.locked():
            state = self.status()
            if not state['can_stop']:
                raise RuntimeError('没有可停止的唯一 start_all 进程；' + state['text'])
            old = state['processes'][0]
            # pidfd pins process identity across PID reuse. Never signal a PID
            # read blindly from boot.pid or kill the process group (posture trap).
            fd = os.pidfd_open(old['pid'])
            try:
                current = process_info(old['pid'])
                if not runs_script(current, self.script) or current['start'] != old['start']:
                    raise RuntimeError('进程已变化，请刷新后重试')
                signal.pidfd_send_signal(fd, signal.SIGTERM)
            finally:
                os.close(fd)
            self.save_metadata({'pid': old['pid'], 'start': old['start'], 'stop_at': time.time(),
                                'script': str(self.script)})

    def log_tail(self):
        try:
            stat = self.log_path.stat()
            key = (stat.st_ino, stat.st_size, stat.st_mtime_ns)
            if key != self._tail_key:
                with self.log_path.open('rb') as stream:
                    stream.seek(max(0, stat.st_size - 24000))
                    self._tail_text = stream.read().decode(errors='replace')
                self._tail_key = key
            return self._tail_text
        except OSError:
            return ''

    def stage(self):
        stages = re.findall(r'=== \[([\d.]+)/8\] (.*?) ===', self.log_tail())
        return ('脚本日志 %s/8 · %s' % stages[-1]) if stages else '等待脚本阶段日志'


class StackPreflight(QThread):
    checked = pyqtSignal(object)

    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager

    def run(self):
        try:
            missing = self.manager.local_checks()
            hosts = {}
            if not missing:
                for host in ('192.168.234.1', '192.168.234.234'):
                    try:
                        hosts[host] = subprocess.run(['ping', '-n', '-c', '1', '-W', '1', host],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=2).returncode == 0
                    except (OSError, subprocess.TimeoutExpired):
                        hosts[host] = False
            self.checked.emit({'ok': not missing and bool(hosts) and all(hosts.values()),
                               'missing': missing, 'hosts': hosts})
        except Exception as exc:
            self.checked.emit({'ok': False, 'missing': [str(exc)], 'hosts': {}})
