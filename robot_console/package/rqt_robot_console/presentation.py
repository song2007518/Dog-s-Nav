"""Console layout and visual components; no device commands are issued here."""
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (QWidget, QLabel, QVBoxLayout, QHBoxLayout, QComboBox,
    QLineEdit, QCheckBox, QSplitter, QTabWidget, QPlainTextEdit, QTableWidget,
    QHeaderView, QAbstractItemView, QScrollArea, QShortcut, QProgressBar, QSizePolicy)

from .canvas import MapCanvas, ScanView, TeleopPad
from .stack_ui import build_stack_ui


STYLE = '''
QWidget {background:#0e1724;color:#dce7f3;font-family:"Noto Sans CJK SC";font-size:12px;}
QLabel {background:transparent;}
QLabel#title {font-size:25px;font-weight:700;color:#f3f7fc;}
QLabel#eyebrow {font-size:10px;font-weight:600;color:#5fd5c4;letter-spacing:2px;}
QLabel#subtitle, QLabel#hint {color:#8fa5bc;font-size:11px;}
QLabel#brand {background:#173d40;color:#74eddb;border:1px solid #26605c;border-radius:12px;font-size:27px;font-weight:700;}
QLabel#banner {background:#142b35;color:#a9d8dc;border:1px solid #264752;border-radius:8px;padding:8px 12px;}
QWidget#metricCard {background:#162335;border:1px solid #293b51;border-radius:10px;}
QLabel#metricTitle {color:#92a9bf;font-size:11px;}
QLabel#metricValue {font-size:19px;font-weight:600;color:#dceaf7;}
QLabel#metricDetail {color:#829ab2;font-size:10px;}
QLabel#metricValue[tone="good"] {color:#68dfc7;}
QLabel#metricValue[tone="warn"] {color:#f4bd74;}
QLabel#metricValue[tone="bad"] {color:#f391a1;}
QGroupBox {background:#121f2f;border:1px solid #27394e;border-radius:9px;margin-top:15px;padding:14px 9px 9px;font-weight:600;}
QGroupBox::title {subcontrol-origin:margin;left:12px;padding:0 5px;color:#adbed2;}
QPushButton {background:#21334a;border:1px solid #344b65;border-radius:6px;padding:7px 10px;font-weight:500;}
QPushButton:hover {background:#2b435f;border-color:#5d8baa;}
QPushButton:pressed, QPushButton:checked {background:#20574f;border-color:#45c8ae;color:#a9fbe9;}
QPushButton:disabled {color:#67798c;background:#192636;border-color:#28394c;}
QPushButton#primary {background:#167f71;color:#efffff;border-color:#35ad99;font-weight:600;}
QPushButton#primary:hover {background:#209a89;}
QPushButton#primary:disabled {background:#192636;color:#67798c;border-color:#28394c;}
QPushButton#stop {background:#743244;border:1px solid #a85065;color:#fff1f3;font-weight:700;}
QPushButton#stop:hover {background:#984056;}
QPushButton#stop:disabled {background:#192636;color:#67798c;border-color:#28394c;}
QLineEdit,QComboBox,QDoubleSpinBox {background:#0e1b2b;border:1px solid #344a62;border-radius:5px;padding:6px;selection-background-color:#286d68;}
QComboBox QAbstractItemView {background:#1b2c41;selection-background-color:#286e66;}
QTabWidget::pane {background:#121f2f;border:1px solid #293b51;border-radius:7px;}
QTabBar::tab {background:#142235;color:#9db2c8;padding:11px 12px;}
QTabBar::tab:selected {background:#1c3545;color:#72e4cf;border-bottom:2px solid #4ed7bd;}
QTableWidget {background:#132133;alternate-background-color:#17283b;gridline-color:#263a4e;border:1px solid #2b4056;border-radius:5px;selection-background-color:#265e65;}
QHeaderView::section {background:#20344a;color:#b6cadc;border:0;padding:8px 5px;font-size:11px;}
QTableWidget::item {padding:5px;}
QPlainTextEdit {background:#0b1420;color:#9cb1c6;border:1px solid #26394d;border-radius:6px;font-family:"Noto Sans Mono CJK SC";font-size:10px;}
QScrollBar:vertical {background:#132031;width:7px;margin:0;} QScrollBar::handle:vertical {background:#39526b;min-height:24px;border-radius:3px;}
QScrollBar:horizontal {background:#132031;height:7px;margin:0;} QScrollBar::handle:horizontal {background:#39526b;min-width:24px;border-radius:3px;}
QScrollBar::add-line,QScrollBar::sub-line {height:0;width:0;}
QCheckBox {spacing:7px;}
QToolTip {background:#22344a;color:white;border:1px solid #56718b;}
QSplitter::handle {background:#0e1724;width:10px;height:10px;}
QProgressBar {background:#21374a;border:0;border-radius:3px;max-height:6px;}
QProgressBar::chunk {background:#51ccb4;border-radius:3px;}
QLabel#routeSummary {color:#bed1e3;font-size:12px;padding:3px 0;}
QLabel#footer {color:#8fa5bc;font-size:10px;}
'''


class MetricCard(QWidget):
    def __init__(self, title, parent=None):
        super().__init__(parent)
        self.setObjectName('metricCard')
        self.setAttribute(Qt.WA_StyledBackground, True)
        self.setMinimumWidth(145)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(3)
        self.title = QLabel(title)
        self.title.setObjectName('metricTitle')
        self.value = QLabel('—')
        self.value.setObjectName('metricValue')
        self.detail = QLabel('等待数据')
        self.detail.setObjectName('metricDetail')
        for widget in (self.title, self.value, self.detail):
            layout.addWidget(widget)

    def setText(self, text):
        title, _, value = text.partition('\n')
        self.title.setText(title)
        self.value.setText(value)

    def set_detail(self, detail, tone='neutral'):
        self.detail.setText(detail)
        if self.value.property('tone') != tone:
            self.value.setProperty('tone', tone)
            self.value.style().unpolish(self.value)
            self.value.style().polish(self.value)


def scroll(widget):
    area = QScrollArea()
    area.setWidgetResizable(True)
    area.setFrameShape(QScrollArea.NoFrame)
    area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    area.setWidget(widget)
    return area


def build_ui(w):
    root = QVBoxLayout(w)
    root.setContentsMargins(18, 14, 18, 12)
    root.setSpacing(12)
    head = QHBoxLayout()
    head.setSpacing(12)
    brand = QLabel('航')
    brand.setObjectName('brand')
    brand.setAlignment(Qt.AlignCenter)
    brand.setFixedSize(54, 54)
    head.addWidget(brand)
    titles = QVBoxLayout()
    titles.setSpacing(1)
    eye = QLabel('ZSL-1W  /  ROBOT OPERATIONS')
    eye.setObjectName('eyebrow')
    title = QLabel('智航 · 机器狗任务控制台')
    title.setObjectName('title')
    titles.addWidget(eye)
    titles.addWidget(title)
    head.addLayout(titles)
    head.addStretch()
    w.mode = QComboBox()
    w.mode.addItems(['离线演示', '实机连接'])
    w.mode.currentIndexChanged.connect(w.switch_mode)
    head.addWidget(w.mode)
    w.presentation_btn = w.button('展示模式', lambda: None)
    w.presentation_btn.setCheckable(True)
    w.presentation_btn.toggled.connect(w.set_presentation)
    w.presentation_btn.setToolTip('收起日志与高级设置，保留数据来源标识；F11 切换')
    head.addWidget(w.presentation_btn)
    head.addWidget(w.button('保存截图', w.save_snapshot))
    w.stop_btn = w.button('■ 停止任务  F12', w.stop_task, stop=True)
    w.stop_btn.setToolTip('仅停止官方导航，不替代物理急停，不控制旧 custom_nav。')
    head.addWidget(w.stop_btn)
    root.addLayout(head)

    w.banner = QLabel()
    w.banner.setObjectName('banner')
    w.banner.setWordWrap(True)
    root.addWidget(w.banner)
    metrics = QHBoxLayout()
    metrics.setSpacing(12)
    w.metrics = {}
    for key, title in [('connection', '通信连接'), ('loc', '定位状态'), ('mapping', '地图建图'), ('nav', '巡航任务')]:
        card = MetricCard(title)
        w.metrics[key] = card
        metrics.addWidget(card, 1)
    root.addLayout(metrics)

    w.split = QSplitter(Qt.Horizontal)
    left = QWidget()
    ll = QVBoxLayout(left)
    ll.setContentsMargins(0, 0, 0, 0)
    ll.setSpacing(10)
    maps = w.group('01   地图工作区', ll)
    w.map_choice = QComboBox()
    w.map_choice.addItems(['演示办公室', '已有实机地图（本地）'])
    w.map_choice.currentIndexChanged.connect(w.choose_map)
    w.map_choice.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
    w.map_choice.setMinimumContentsLength(10)
    maps.addWidget(w.map_choice)
    w.map_details = QLabel()
    w.map_details.setObjectName('hint')
    w.map_details.setWordWrap(True)
    maps.addWidget(w.map_details)
    row = QHBoxLayout()
    row.addWidget(w.button('打开地图', w.open_map))
    row.addWidget(w.button('导出地图', w.export_map))
    maps.addLayout(row)
    mapping = w.group('02   建图与定位', ll)
    w.start_map_btn = w.button('开始建图', w.start_mapping)
    w.stop_map_btn = w.button('结束建图并保存', w.stop_mapping)
    mapping.addWidget(w.start_map_btn)
    mapping.addWidget(w.stop_map_btn)
    w.load_loc_btn = w.button('加载所选地图定位', w.load_localization)
    w.reset_loc_btn = w.button('重置定位', w.reset_localization)
    mapping.addWidget(w.load_loc_btn)
    mapping.addWidget(w.reset_loc_btn)
    w.loc_note = QLabel('定位丢失时停止任务，进入手动待接管。')
    w.loc_note.setObjectName('hint')
    w.loc_note.setWordWrap(True)
    mapping.addWidget(w.loc_note)
    tips = w.group('03   操作指引', ll)
    guide = QLabel('01  选择地图与巡航点\n02  检查位置和朝向\n03  保存路线，启动任务')
    guide.setObjectName('hint')
    tips.addWidget(guide)
    w.preview_btn = w.button('预览模拟路径', w.preview_demo_route)
    tips.addWidget(w.preview_btn)
    w.reset_scene_btn = w.button('重置演示场景', w.reset_demo)
    tips.addWidget(w.reset_scene_btn)
    w.stack_entry = w.button('系统启动 · 打开', w.show_stack_page, primary=True)
    tips.addWidget(w.stack_entry)
    ll.addStretch()
    left_scroll = scroll(left)
    left_scroll.setMinimumWidth(200)
    left_scroll.setMaximumWidth(250)

    center = QWidget()
    cl = QVBoxLayout(center)
    cl.setContentsMargins(0, 0, 0, 0)
    cl.setSpacing(8)
    bar = QHBoxLayout()
    map_title = QLabel('全局地图')
    map_title.setStyleSheet('font-size:15px;font-weight:600;color:#dce7f3;')
    bar.addWidget(map_title)
    bar.addStretch()
    w.add_btn = w.button('＋ 巡航点', lambda: None)
    w.add_btn.setCheckable(True)
    bar.addWidget(w.add_btn)
    bar.addWidget(w.button('适应窗口', lambda: w.canvas.fit_map()))
    cl.addLayout(bar)
    w.map_meta = QLabel()
    w.map_meta.setObjectName('hint')
    w.map_meta.setMinimumWidth(0)
    w.map_meta.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    cl.addWidget(w.map_meta)
    w.canvas = MapCanvas()
    w.canvas.setMinimumSize(300, 300)
    w.add_btn.toggled.connect(w.canvas.set_add_mode)
    w.canvas.pointAdded.connect(w.add_point)
    w.canvas.pointSelected.connect(w.select_point)
    w.canvas.pointMoved.connect(w.move_point)
    w.canvas.cursorWorld.connect(lambda x, y: w.coords.setText('光标  X %.2f  /  Y %.2f m' % (x, y)))
    cl.addWidget(w.canvas, 1)
    legend = QLabel('● 巡航点   ┄ 点位顺序   ━ 模拟路径   ▲ 机器人   · 激光')
    w.map_legend = legend
    legend.setObjectName('hint')
    legend.setWordWrap(True)
    cl.addWidget(legend)
    w.coords = QLabel('拖动画布平移 · 滚轮缩放 · 点选后拖出朝向')
    w.coords.setObjectName('hint')
    cl.addWidget(w.coords)

    right = QWidget()
    right.setMinimumWidth(310)
    right.setMaximumWidth(370)
    rl = QVBoxLayout(right)
    rl.setContentsMargins(0, 0, 0, 0)
    rl.setSpacing(10)
    w.tabs = QTabWidget()
    rl.addWidget(w.tabs, 1)
    route = QWidget()
    route_layout = QVBoxLayout(route)
    route_layout.setContentsMargins(12, 12, 12, 12)
    route_layout.setSpacing(8)
    w.route_name = QLineEdit('patrol_01')
    w.route_name.setPlaceholderText('路线 ID')
    w.route_name.textEdited.connect(w.mark_dirty)
    route_layout.addWidget(w.route_name)
    w.route_summary = QLabel()
    w.route_summary.setObjectName('routeSummary')
    route_layout.addWidget(w.route_summary)
    w.progress = QProgressBar()
    w.progress.setTextVisible(False)
    route_layout.addWidget(w.progress)
    nav = QHBoxLayout()
    w.start_nav_btn = w.button('▶ 开始巡航', w.start_nav, primary=True)
    w.pause_btn = w.button('暂停', w.pause_nav)
    nav.addWidget(w.start_nav_btn, 2)
    nav.addWidget(w.pause_btn, 1)
    route_layout.addLayout(nav)
    w.table = QTableWidget(0, 4)
    w.table.setHorizontalHeaderLabels(['巡航点', 'X / m', 'Y / m', '朝向 °'])
    w.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    w.table.verticalHeader().setVisible(False)
    w.table.verticalHeader().setDefaultSectionSize(38)
    w.table.setAlternatingRowColors(True)
    w.table.setShowGrid(False)
    w.table.setSelectionBehavior(QAbstractItemView.SelectRows)
    w.table.setSelectionMode(QAbstractItemView.SingleSelection)
    w.table.setMinimumHeight(180)
    w.table.itemSelectionChanged.connect(w.table_selection)
    w.table.cellChanged.connect(w.edit_point)
    route_layout.addWidget(w.table, 1)
    edits = QHBoxLayout()
    for title, fn in [('上移', lambda: w.reorder(-1)), ('下移', lambda: w.reorder(1)), ('删除', w.delete_point)]:
        edits.addWidget(w.button(title, fn))
    route_layout.addLayout(edits)
    files = QHBoxLayout()
    files.addWidget(w.button('保存路线', w.save_route_dialog))
    files.addWidget(w.button('加载路线', w.load_route_dialog))
    route_layout.addLayout(files)
    w.route_note = QLabel('双击表格编辑坐标；连线表示点位顺序。')
    w.route_note.setObjectName('hint')
    w.route_note.setWordWrap(True)
    route_layout.addWidget(w.route_note)
    w.tabs.addTab(scroll(route), '巡航任务')

    control = QWidget()
    tl = QVBoxLayout(control)
    w.manual_btn = w.button('切换到手动接管', w.manual_takeover)
    tl.addWidget(w.manual_btn)
    w.pad = TeleopPad()
    w.pad.command.connect(w.teleop)
    tl.addWidget(w.pad)
    w.manual_note = QLabel('键盘与近障停车用于离线演示。实机速度通道尚未部署。')
    w.manual_note.setObjectName('hint')
    w.manual_note.setWordWrap(True)
    tl.addWidget(w.manual_note)
    w.fail_btn = w.button('模拟定位丢失 → 停车待接管', w.inject_loc_failure)
    w.obstacle_btn = w.button('前方添加模拟障碍', w.inject_obstacle)
    w.example_btn = w.button('重置演示场景与路线', w.reset_demo)
    for b in (w.fail_btn, w.obstacle_btn, w.example_btn):
        tl.addWidget(b)
    tl.addStretch()
    w.tabs.addTab(scroll(control), '接管演示')

    w.stack_tab_index = w.tabs.addTab(scroll(build_stack_ui(w)), '系统启动')

    device = QWidget()
    dl = QVBoxLayout(device)
    conn = w.group('设备通信', dl)
    w.url = QLineEdit('ws://192.168.234.234:10010')
    w.connect_btn = w.button('连接设备', w.toggle_connection, primary=True)
    w.fetch_btn = w.button('读取 / 重试设备地图', w.request_maps)
    w.write_enabled = QCheckBox('允许真实操作')
    w.write_enabled.setToolTip('仅允许主动点击后的操作；不会自动站立或发布速度。')
    w.write_enabled.toggled.connect(w.refresh)
    w.ros_button = w.button('启动 ROS 只读监测', w.toggle_ros)
    for obj in (w.url, w.connect_btn, w.fetch_btn, w.write_enabled, w.ros_button):
        conn.addWidget(obj)
    w.ros_note = QLabel('地图 / 位姿 / 激光：等待订阅')
    w.ros_note.setWordWrap(True)
    w.ros_note.setObjectName('hint')
    conn.addWidget(w.ros_note)
    advanced = w.group('地图数据与路线', dl)
    w.encoding = QComboBox()
    w.encoding.addItems(['占用栅格 -1..100', 'PGM 灰度 0..255'])
    w.encoding.currentIndexChanged.connect(w.redecode_map)
    w.live_map = QCheckBox('每 3 秒读取当前建图栅格')
    w.ros_map = QCheckBox('显示 ROS /map（只读预览）')
    for obj in (w.encoding, w.live_map, w.ros_map):
        advanced.addWidget(obj)
    advanced.addWidget(w.button('输入坐标添加点', w.numeric_point))
    advanced.addWidget(w.button('导出 WebSocket 请求', w.export_requests))
    w.upload_btn = w.button('保存路线到机器狗', w.upload_route)
    w.read_routes_btn = w.button('读取机器狗已有路线', lambda: w.send('get_all_paths_by_mapid', w.grid.map_id))
    w.nav_owner = QCheckBox('确认旧导航已停用，允许官方导航')
    w.nav_owner.toggled.connect(w.refresh)
    for obj in (w.upload_btn, w.read_routes_btn, w.nav_owner):
        advanced.addWidget(obj)
    dl.addStretch()
    w.tabs.addTab(scroll(device), '设置')

    w.scan_view = ScanView()
    w.scan_view.setMinimumHeight(180)
    w.scan_view.setMaximumHeight(230)
    rl.addWidget(w.scan_view)
    w.split.addWidget(left_scroll)
    w.split.addWidget(center)
    w.split.addWidget(right)
    w.split.setChildrenCollapsible(False)
    w.split.setStretchFactor(1, 1)
    w.split.setSizes([220, 750, 340])
    root.addWidget(w.split, 1)

    footer = QHBoxLayout()
    w.log_toggle = w.button('运行日志', lambda: None)
    w.log_toggle.setCheckable(True)
    w.log_toggle.setChecked(True)
    footer.addWidget(w.log_toggle)
    w.last_event = QLabel('准备就绪')
    w.last_event.setObjectName('footer')
    w.last_event.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
    w.last_event.setMinimumWidth(0)
    footer.addWidget(w.last_event, 1)
    footer.addWidget(w.button('导出诊断', w.export_log))
    w.clock_label = QLabel()
    w.clock_label.setObjectName('footer')
    footer.addWidget(w.clock_label)
    root.addLayout(footer)
    w.logs = QPlainTextEdit()
    w.logs.setReadOnly(True)
    w.logs.setMaximumBlockCount(1000)
    w.logs.setFixedHeight(78)
    w.log_toggle.toggled.connect(w.logs.setVisible)
    root.addWidget(w.logs)
    w._stop_shortcut = QShortcut(QKeySequence('F12'), w)
    w._stop_shortcut.activated.connect(w.stop_task)
    w._presentation_shortcut = QShortcut(QKeySequence('F11'), w)
    w._presentation_shortcut.activated.connect(w.presentation_btn.toggle)
