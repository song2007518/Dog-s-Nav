"""User actions for the existing robot workspace launcher."""
import os
import time
from PyQt5.QtCore import QTimer, QUrl
from PyQt5.QtGui import QDesktopServices
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QCheckBox, QPlainTextEdit, QHBoxLayout
from .stack import StackPreflight


def build_stack_ui(w):
    page = QWidget()
    layout = QVBoxLayout(page)
    layout.setSpacing(9)
    title = QLabel('START ALL  /  系统启动')
    title.setObjectName('eyebrow')
    layout.addWidget(title)
    w.stack_label = QLabel('未启动')
    w.stack_label.setWordWrap(True)
    w.stack_label.setStyleSheet('font-size:16px;font-weight:600;color:#72e4cf;')
    layout.addWidget(w.stack_label)
    w.stack_stage = QLabel('待执行启动检查')
    w.stack_stage.setObjectName('hint')
    w.stack_stage.setWordWrap(True)
    layout.addWidget(w.stack_stage)
    steps = QLabel('通信 / 驱动 → 站立 / 定位\n地图 / 导航 → RViz / 相机守护')
    layout.addWidget(steps)
    note = QLabel('重启导航组件并自动站立，掉线或定位失败后可能再次站立；RViz 目标可触发行走。运行时锁定官方连接。')
    note.setObjectName('hint')
    note.setWordWrap(True)
    layout.addWidget(note)
    w.stack_check_note = QLabel('检查依赖与板卡连通性；授时、定位、姿态需实机核验。')
    w.stack_check_note.setWordWrap(True)
    w.stack_check_note.setObjectName('hint')
    layout.addWidget(w.stack_check_note)
    w.stack_check_btn = w.button('检查启动条件（只读）', lambda: w.check_stack(False))
    layout.addWidget(w.stack_check_btn)
    w.stack_consent = QCheckBox('场地已清空，允许自动站立与守护')
    w.stack_consent.setToolTip('确认机器狗周围无人、未运行其他运动任务；脚本自愈期间可能再次站立。')
    w.stack_consent.toggled.connect(w.refresh)
    layout.addWidget(w.stack_consent)
    w.stack_start_btn = w.button('▶ 一键启动', w.start_stack, primary=True)
    w.stack_stop_btn = w.button('■ 蹲下 → 阻尼', w.stop_stack, stop=True)
    controls = QHBoxLayout()
    controls.addWidget(w.stack_start_btn)
    controls.addWidget(w.stack_stop_btn)
    layout.addLayout(controls)
    w.stack_monitor_btn = w.button('显示 ROS 地图与实时路径', w.enable_stack_monitor)
    layout.addWidget(w.stack_monitor_btn)
    w.stack_cancel_btn = w.button('取消本次启动', w.cancel_stack_start)
    layout.addWidget(w.stack_cancel_btn)
    w.stack_lifetime = QLabel('关闭 rqt 保留守护。收工需核验姿态；不替代遥控器急停。')
    w.stack_lifetime.setObjectName('hint')
    w.stack_lifetime.setWordWrap(True)
    layout.addWidget(w.stack_lifetime)
    logrow = QHBoxLayout()
    logrow.addWidget(QLabel('原脚本日志 · 不作为动作完成凭据'))
    logrow.addStretch()
    layout.addLayout(logrow)
    w.stack_logs = QPlainTextEdit()
    w.stack_logs.setReadOnly(True)
    w.stack_logs.setMinimumHeight(100)
    w.stack_logs.setMaximumBlockCount(400)
    layout.addWidget(w.stack_logs, 1)
    layout.addWidget(w.button('打开日志目录', lambda: QDesktopServices.openUrl(QUrl.fromLocalFile(str(w.stack.log_dir)))))
    path = QLabel('原脚本：' + w.stack.script.parent.parent.name + ' / scripts / start_all.sh')
    path.setToolTip(str(w.stack.script))
    path.setObjectName('hint')
    path.setWordWrap(True)
    path.setTextInteractionFlags(path.textInteractionFlags() | 1)
    layout.addWidget(path)
    return page


class StackActions:
    def stack_locked(self):
        return self._stack_pending or self.stack_state['busy']

    def show_stack_page(self):
        self.tabs.setCurrentIndex(self.stack_tab_index)

    def prepare_stack_page(self):
        self.dirty = False
        self.mode.setCurrentIndex(1)
        self.show_stack_page()
        self.log('系统启动页已打开。点击启动前不会连接设备或发送姿态指令。')

    def poll_stack(self):
        if self._closed:
            return
        previous = self.stack_state
        self.stack_state = self.stack.status()
        if self.stack_state['busy'] and not previous['busy']:
            self.write_enabled.setChecked(False)
            self.nav_owner.setChecked(False)
            if self.worker:
                self.disconnect_device()
                self.log('检测到 start_all / 自定义导航，已让出 WebSocket。')
        if self.stack_state['text'] != previous['text']:
            self.log('系统启动：' + self.stack_state['text'])
        tail = self.stack.log_tail()
        if tail != self.stack_logs.toPlainText():
            bar = self.stack_logs.verticalScrollBar()
            at_bottom = bar.value() >= bar.maximum() - 2
            position = bar.value()
            self.stack_logs.setPlainText(tail)
            bar.setValue(bar.maximum() if at_bottom else position)
        self.refresh()

    def check_stack(self, launch=False):
        if self._stack_check and self._stack_check.isRunning():
            return
        self._stack_pending = bool(launch)
        check = StackPreflight(self.stack, self)
        self._stack_check = check
        check.checked.connect(self.stack_checked)
        check.finished.connect(self.refresh)
        self.stack_check_note.setText('正在检查本地文件与板卡连通性…')
        check.start()
        self.refresh()

    def stack_checked(self, result):
        if self._closed:
            return
        lines = []
        if result['missing']:
            lines.append('缺少依赖：' + '、'.join(result['missing']))
        else:
            lines.append('本地依赖齐全')
        lines += [host + ('：可达' if ok else '：不可达') for host, ok in result['hosts'].items()]
        lines.append('授时、定位、姿态：尚未核验')
        self.stack_check_note.setText('\n'.join(lines))
        self.log('启动检查：' + '；'.join(lines))
        if not result['ok']:
            self._stack_pending = False
        elif self._stack_pending:
            # The choice/mode may have changed while the read-only check ran.
            if not self.live or not self.stack_consent.isChecked():
                self.cancel_stack_start()
                return
            self.stack_state = self.stack.status()
            if self.stack_state['busy']:
                self._stack_pending = False
                self.error(self.stack_state['text'])
                return
            if self.worker and (self.states['nav'] not in ('StandBy','Cancelled','Succeed','Failed')
                    or time.monotonic()-self.status_times.get('nav', 0) > 12):
                self._stack_pending = False
                self.error('官方导航尚未确认停止，先停止任务并刷新状态')
                return
            if any(not func.startswith('get_') for func, value in self.pending.values()):
                self._stack_pending = False
                self.error('设备操作仍在等待响应，请先核验结果')
                return
            self.disconnect_device()
            self.stop_ros()
            self._stack_deadline = time.monotonic() + 10
            self._finish_stack_start()
        self.refresh()

    def start_stack(self):
        if not self.live or not self.stack_consent.isChecked():
            self.error('请切换到实机模式，并确认场地安全与自动站立权限')
            return
        if self.stack_locked():
            self.error('已有启动流程或导航组件运行，请先查看系统启动状态')
            return
        if self.worker and (self.states['nav'] not in ('StandBy','Cancelled','Succeed','Failed')
                or time.monotonic()-self.status_times.get('nav',0)>12):
            self.error('请先停止官方导航并刷新状态，再启动 start_all')
            return
        if any(not func.startswith('get_') for func,value in self.pending.values()):
            self.error('设备操作尚未确认完成，请先核验结果')
            return
        self.check_stack(True)

    def _finish_stack_start(self):
        if self._closed or not self._stack_pending:
            return
        if not self.live or not self.stack_consent.isChecked():
            self.cancel_stack_start()
            return
        if any(thread.isRunning() for thread in self.retired):
            if time.monotonic() > self._stack_deadline:
                self._stack_pending = False
                self.error('通信线程未退出，已取消启动；请检查连接后重试')
            else:
                QTimer.singleShot(100, self._finish_stack_start)
            return
        try:
            pid = self.stack.start()
            self.log('start_all 已启动，PID %s；正在执行原脚本，设备就绪状态需核验。' % pid)
        except Exception as exc:
            self.error('启动失败：' + str(exc))
        self._stack_pending = False
        self.stack_consent.setChecked(False)
        self.poll_stack()

    def cancel_stack_start(self):
        self._stack_pending = False
        self.log('已取消本次启动；检查结果仅供查看。')
        self.refresh()

    def stop_stack(self):
        if self._stack_pending:
            self.cancel_stack_start()
            return
        self.stack_state=self.stack.status()
        if self.stack_state['state']=='stopping':
            self.log('收工流程正在执行，未重复发送信号；请查看日志并观察机器狗。')
            return
        try:
            self.stack.stop()
            self.log('已请求原脚本执行收工：禁用运动 → 蹲下 → 阻尼 → 清理。请观察实际姿态。')
        except Exception as exc:
            self.error('无法执行收工：' + str(exc) + '；需要紧急停车时请使用遥控器。')
        self.poll_stack()

    def enable_stack_monitor(self):
        if not self.live or self._stack_pending:
            return
        if os.environ.get('ROS_DOMAIN_ID')!='24' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp':
            self.error('请使用 start_console.sh --stack 打开窗口，再读取机器人 ROS 数据')
            return
        if not self.editing_allowed() or not self.discard_ok():
            return
        from .core import GridMap
        self.install_map(GridMap('等待 ROS /map',100,80,.1,(0,0,0),[-1]*8000,source='empty'))
        self.ros_map.setChecked(True)
        if not self.monitor:
            self.toggle_ros()
        self.log('只读订阅 /map、位姿、激光及 /custom_nav/path；没有下发目标或速度。')

    def refresh_stack(self):
        locked = self.stack_locked()
        checking = self._stack_check is not None and self._stack_check.isRunning()
        self.stack_label.setText('启动检查 / 正在让出连接' if self._stack_pending else self.stack_state['text'])
        processes = self.stack_state['processes']
        self.stack_stage.setText(('PID %s\n%s' % (processes[0]['pid'], self.stack.stage())) if processes
                                else '实机未验收 · 不随 rqt 自动启动')
        self.stack_consent.setEnabled(self.live and not locked)
        self.stack_start_btn.setEnabled(self.live and not locked and not checking and self.stack_consent.isChecked())
        self.stack_stop_btn.setEnabled(self.stack_state['can_stop'])
        self.stack_cancel_btn.setVisible(self._stack_pending)
        self.stack_check_btn.setEnabled(not checking and not locked)
        self.stack_monitor_btn.setEnabled(self.live and not self._stack_pending)
        if self._stack_pending:
            self.ros_button.setEnabled(False)
        self.stack_entry.setText('系统启动 · ' + ('运行中' if processes else '检查中' if self._stack_pending else '打开'))
        if locked:
            self.banner.setText('●  START ALL  /  ' + self.stack_state['text'] + '   ·   官方 WebSocket 已锁定，关闭 rqt 会保留守护进程')
            for widget in (self.connect_btn, self.write_enabled, self.nav_owner, self.fetch_btn,
                           self.start_map_btn, self.stop_map_btn, self.load_loc_btn, self.reset_loc_btn,
                           self.upload_btn, self.read_routes_btn, self.start_nav_btn, self.pause_btn):
                widget.setEnabled(False)
            if self.live:
                now=time.monotonic()
                pose_fresh=now-self.last_pose<1.5
                nav_fresh=now-self.last_custom_nav<2
                self.metrics['connection'].setText('通信连接\n'+('ROS 只读' if self.monitor else '脚本管理中'))
                self.metrics['connection'].set_detail('WebSocket 由原脚本管理')
                self.metrics['loc'].setText('定位\n'+('位姿有效' if pose_fresh else '等待 ROS 位姿'))
                self.metrics['loc'].set_detail(self.ros_status.get('odom_type','尚无实时定位来源') if pose_fresh else '未核验定位', 'good' if pose_fresh else 'warn')
                self.metrics['mapping'].setText('地图\n'+('ROS 地图已接收' if self.grid.source=='ros' else '等待 ROS 地图'))
                self.metrics['mapping'].set_detail('只读显示，不代表建图正在运行')
                self.metrics['nav'].setText('自定义导航\n'+('收到状态' if nav_fresh else '等待状态'))
                self.metrics['nav'].set_detail('/nav_status · '+('数据新鲜' if nav_fresh else '未收到 / 已过期'), 'good' if nav_fresh else 'warn')
        self.map_legend.setText('● 巡航点   ┄ 点位顺序   ━ '+('ROS 路径（节点输出）' if self.live else '模拟路径')+'   ▲ 机器人   · 激光')
        self.mode.setEnabled(not locked)
        self.stop_btn.setText('■ 收工 / 取消启动  F12' if locked else '■ 停止任务  F12')
        self.stop_btn.setToolTip('请求 start_all 收工：蹲下、阻尼；不替代物理急停。' if locked else
                                '停止官方导航或离线模拟任务；不替代物理急停。')
