import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer, QEvent
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (QApplication,QWidget,QVBoxLayout,QHBoxLayout,QGridLayout,QLabel,
    QPushButton,QComboBox,QLineEdit,QCheckBox,QSplitter,QTabWidget,QGroupBox,QPlainTextEdit,
    QTableWidget,QTableWidgetItem,QHeaderView,QAbstractItemView,QDoubleSpinBox,QFileDialog,
    QMessageBox,QShortcut,QInputDialog,QScrollArea)

from .core import (GridMap, Waypoint, demo_map, demo_plan, load_route, save_route, finite,
                   validate_points, path_payload, request, atomic_json, yaw_of)
from .canvas import MapCanvas,ScanView,TeleopPad
from .transport import WsWorker,RosMonitor
from .planning import segment_clear, ROBOT_RADIUS
from .stack import StackManager
from .stack_ui import StackActions


def assets_dir():
    source=Path(__file__).resolve().parents[1]/'assets'
    if source.exists(): return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('rqt_robot_console'))/'assets'


from .presentation import STYLE, build_ui


class Console(StackActions, QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setWindowTitle('智航 · 机器狗任务控制台')
        self.setObjectName('RobotConsole')
        self.setAttribute(Qt.WA_StyledBackground,True)
        self.setStyleSheet(STYLE)
        self.resize(1440,900)
        self.worker=None
        self.retired=[]
        self.monitor=None
        self.pending={}
        self.live=False
        self.online=False
        self.last_response=0
        self.last_pose=0
        self.last_scan=0
        self.last_path=0
        self.last_custom_nav=0
        self.last_odom_source=0
        self.ros_status={}
        self.status_times={}
        self.states={'mapping':'MappingReady','loc':'ContinuousLoc','nav':'StandBy'}
        self.loaded_map_id=None
        self.raw_maps={}
        self.map_read_complete=False
        self.map_retry_at=0.
        self.map_retry_delay=2.
        self.known_paths={}
        self.grid=demo_map()
        self.points=[]
        self.dirty=False
        self.table_updating=False
        self.robot=(-3.8,-2.6,0.0)
        self.demo_mode='待命'
        self.demo_target=0
        self.demo_path=[]
        self.demo_cmd=(0.,0.)
        self.demo_block=False
        self.log_lines=[]
        self._closed=False
        self.stack=StackManager()
        self.stack_state=self.stack.status()
        self._stack_pending=False
        self._stack_check=None
        self._build()
        self.canvas.set_grid(self.grid)
        self.installEventFilter(self)
        QApplication.instance().installEventFilter(self)
        self.timer=QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.poll_timer=QTimer(self)
        self.poll_timer.timeout.connect(self.poll)
        self.poll_timer.start(4000)
        self.map_timer=QTimer(self)
        self.map_timer.timeout.connect(self.refresh_live_map)
        self.map_timer.start(3000)
        self.stack_timer=QTimer(self)
        self.stack_timer.timeout.connect(self.poll_stack)
        self.stack_timer.start(1000)
        QTimer.singleShot(0,self.canvas.fit_map)
        self.load_example()
        self.log('就绪：离线演示模式，未连接设备。示例轨迹由本地模拟器生成。')
        self.refresh()
        if os.environ.get('ROBOT_CONSOLE_STACK')=='1' or self.stack_state['busy']:
            QTimer.singleShot(500,self.prepare_stack_page)
        elif os.environ.get('ROBOT_CONSOLE_CONNECT_READONLY')=='1':
            QTimer.singleShot(500,self.connect_readonly)
        if os.environ.get('ROBOT_CONSOLE_PRESENTATION')=='1':
            QTimer.singleShot(200,self.prepare_presentation)

    def connect_readonly(self):
        self.dirty=False  # Initial generated demo route has no user edits.
        self.mode.setCurrentIndex(1)
        self.stack_state=self.stack.status()
        if self.stack_locked():
            self.show_stack_page()
            self.log('检测到 start_all / 自定义导航，跳过自动 WebSocket 连接。')
            return
        self.toggle_ros()
        self.toggle_connection()

    def button(self,text,fn,primary=False,stop=False):
        b=QPushButton(text)
        b.clicked.connect(lambda checked=False:fn())
        if primary: b.setObjectName('primary')
        if stop: b.setObjectName('stop')
        return b

    def group(self,title,parent_layout):
        box=QGroupBox(title)
        layout=QVBoxLayout(box)
        layout.setSpacing(7)
        parent_layout.addWidget(box)
        return layout

    def _build(self):
        build_ui(self)

    def set_presentation(self, enabled):
        self.presentation_btn.setText('退出展示' if enabled else '展示模式')
        if enabled:
            self._previous_log_visible = self.log_toggle.isChecked()
            self.log_toggle.setChecked(False)
            self.tabs.setCurrentIndex(0)
        else:
            self.log_toggle.setChecked(getattr(self, '_previous_log_visible', True))
        QTimer.singleShot(0, self.canvas.fit_map)

    def prepare_presentation(self):
        self.presentation_btn.setChecked(True)
        if os.environ.get('ROBOT_CONSOLE_CONNECT_READONLY')!='1' and not self.live and self.grid.source=='demo':
            self.preview_demo_route()

    def save_snapshot(self):
        base = next((p for p in Path(__file__).resolve().parents if (p / 'start_console.sh').exists()), Path.cwd())
        destination = base / 'screenshots'
        destination.mkdir(parents=True, exist_ok=True)
        tag = '实机' if self.live else '离线演示' if self.grid.source=='demo' else '本地地图'
        filename = destination / ('任务控制台-%s-%s.png' % (tag, datetime.now().strftime('%Y%m%d-%H%M%S-%f')))
        try:
            if not self.grab().save(str(filename)):
                raise RuntimeError('无法写入截图')
            atomic_json(filename.with_suffix('.json'), {
                'mode': 'robot' if self.live else 'demo' if self.grid.source=='demo' else 'local', 'map_source': self.grid.source,
                'map_id': self.grid.map_id, 'states': self.states if self.live or self.grid.source=='demo' else {},
                'time': datetime.now().isoformat(), 'route_points': len(self.points),
                'stack': {k:self.stack_state[k] for k in ('state','text','exit_code')},
                'note': '运动、激光、任务状态为模拟' if not self.live and self.grid.source=='demo' else
                        '本地地图，无实时设备状态' if not self.live else '实机界面，状态以有效数据为准；脚本运行不等于设备就绪'})
            self.log('截图已保存：' + str(filename))
            return filename
        except Exception as exc:
            self.error('截图保存失败：' + str(exc))
            return None

    def log(self,text):
        line='[%s] %s' % (datetime.now().strftime('%H:%M:%S'),text)
        self.log_lines.append(line)
        self.log_lines=self.log_lines[-3000:]
        self.logs.appendPlainText(line)
        self.last_event.setText(line)
        self.last_event.setToolTip(line)

    def error(self,text):
        self.log('⚠ '+str(text))
        self.route_note.setText(str(text))

    def mark_dirty(self,*args):
        self.dirty=True

    def discard_ok(self):
        if self.dirty:
            return QMessageBox.question(self,'未保存的路线','当前路线尚未保存。确定放弃这些修改？',
                  QMessageBox.Yes|QMessageBox.No,QMessageBox.No)==QMessageBox.Yes
        return True

    def install_map(self,grid,clear=True):
        if clear:
            self.points=[]
            self.dirty=False
        self.demo_path=[]
        self.demo_mode='待命'
        self.demo_target=0
        self.demo_cmd=(0,0)
        self.canvas.path=[]
        self.canvas.robot=None
        self.canvas.scan=[]
        self.grid=grid
        if not self.live and grid.source!='demo':
            self.scan_view.points=[]
            self.scan_view.caption='本地地图 · 无实时激光'
            self.scan_view.update()
        self.canvas.set_grid(grid)
        self.sync_table()
        self.log('地图：%s · %d × %d · %.3f m/格 · frame=%s' %
                 (grid.map_id,grid.width,grid.height,grid.resolution,grid.frame))
        self.refresh()

    def choose_map(self,index):
        if index < 0: return
        if not self.editing_allowed():
            self.restore_map_choice(); return
        if not self.discard_ok():
            self.restore_map_choice()
            return
        try:
            if not self.live:
                if index==0:
                    self.install_map(demo_map())
                    self.robot=(-3.8,-2.6,0)
                else:
                    self.install_map(GridMap.from_yaml(assets_dir()/'robot_map.yaml'))
            else:
                mid=self.map_choice.itemData(index)
                if mid in self.raw_maps:
                    grid=GridMap.from_payload(mid,self.raw_maps[mid],self.map_encoding())
                    self.install_map(grid)
        except Exception as exc:
            self.error(exc)
            self.restore_map_choice()

    def restore_map_choice(self):
        self.map_choice.blockSignals(True)
        if self.live:
            self.map_choice.setCurrentIndex(self.map_choice.findData(self.grid.map_id))
        else:
            self.map_choice.setCurrentIndex(0 if self.grid.source=='demo' else 1)
        self.map_choice.blockSignals(False)

    def map_encoding(self):
        return 'occupancy' if self.encoding.currentIndex()==0 else 'pgm'

    def redecode_map(self):
        if self.live and self.grid.map_id in self.raw_maps:
            self.choose_map(self.map_choice.currentIndex())

    def open_map(self):
        if not self.editing_allowed(): return
        if not self.discard_ok(): return
        path,_=QFileDialog.getOpenFileName(self,'打开 ROS 地图 YAML',str(assets_dir()),'地图 (*.yaml *.yml)')
        if path:
            try: self.install_map(GridMap.from_yaml(path))
            except Exception as exc: self.error(exc)

    def export_map(self):
        path,_=QFileDialog.getSaveFileName(self,'导出地图 YAML + PGM','map.yaml','地图 (*.yaml)')
        if path:
            try:
                self.grid.export(path)
                self.log('地图已导出：'+path)
            except Exception as exc: self.error(exc)

    def add_point(self,x,y,yaw):
        if not self.editing_allowed(): return
        if len(self.points)>=1000:
            self.error('最多 1000 个巡航点'); return
        if not self.grid.valid_point(x,y):
            self.error('请选择地图内已知空闲区域'); return
        names={p.name for p in self.points}
        n=1
        while 'P%02d'%n in names: n+=1
        self.points.append(Waypoint('P%02d'%n,x,y,yaw))
        self.dirty=True
        self.sync_table(len(self.points)-1)
        self.log('新增 %s：(%.2f, %.2f)，朝向 %.1f°' % (self.points[-1].name,x,y,math.degrees(yaw)))

    def numeric_point(self):
        text,ok=QInputDialog.getText(self,'添加巡航点','输入 x, y, 朝向角度，例如：1.0, 2.0, 90')
        if ok:
            try:
                x,y,a=[finite(v.strip()) for v in text.split(',')]
                self.add_point(x,y,math.radians(a))
            except Exception: self.error('请输入三个有限数字，以英文逗号分隔')

    def select_point(self,index):
        self.table.selectRow(index)

    def table_selection(self):
        self.canvas.selected=self.table.currentRow()
        self.canvas.viewport().update()

    def move_point(self,index,x,y):
        if not self.editing_allowed(): return
        if 0<=index<len(self.points):
            if not self.grid.valid_point(x,y):
                self.error('不能将巡航点移动到障碍、未知区域或地图外'); return
            self.points[index].x,self.points[index].y=x,y
            self.dirty=True
            self.sync_table(index)

    def edit_point(self,row,col):
        if self.table_updating or not 0<=row<len(self.points): return
        old=self.points[row]
        try:
            p=Waypoint(self.table.item(row,0).text().strip(),finite(self.table.item(row,1).text()),
                       finite(self.table.item(row,2).text()),math.radians(finite(self.table.item(row,3).text())))
            candidate=self.points[:]
            candidate[row]=p
            validate_points(candidate,self.grid)
            self.points=candidate
            self.dirty=True
        except Exception as exc:
            self.error(exc)
        self.sync_table(row)

    def sync_table(self,selected=None):
        if self.demo_mode not in ('自动巡航','暂停'):
            self.canvas.path=[]
        self.table_updating=True
        self.table.setRowCount(len(self.points))
        for i,p in enumerate(self.points):
            for j,value in enumerate([p.name,'%.3f'%p.x,'%.3f'%p.y,'%.1f'%math.degrees(p.yaw)]):
                self.table.setItem(i,j,QTableWidgetItem(value))
        self.table_updating=False
        self.canvas.points=self.points
        if selected is not None and 0<=selected<len(self.points): self.table.selectRow(selected)
        self.canvas.viewport().update()
        self.refresh()

    def reorder(self,delta):
        if not self.editing_allowed(): return
        i=self.table.currentRow()
        if 0<=i<len(self.points) and 0<=i+delta<len(self.points):
            self.points[i],self.points[i+delta]=self.points[i+delta],self.points[i]
            self.dirty=True
            self.sync_table(i+delta)

    def delete_point(self):
        if not self.editing_allowed(): return
        i=self.table.currentRow()
        if 0<=i<len(self.points):
            self.points.pop(i)
            self.dirty=True
            self.sync_table(min(i,len(self.points)-1))

    def save_route_dialog(self):
        try: validate_points(self.points,self.grid)
        except Exception as exc: self.error(exc); return
        path,_=QFileDialog.getSaveFileName(self,'保存巡航路线',self.route_name.text()+'.json','路线 (*.json)')
        if path:
            try:
                save_route(path,self.grid.map_id,self.route_name.text().strip(),self.points)
                self.dirty=False
                self.log('路线已保存：'+path)
            except Exception as exc: self.error(exc)

    def load_route_dialog(self):
        if not self.editing_allowed(): return
        if not self.discard_ok(): return
        path,_=QFileDialog.getOpenFileName(self,'加载巡航路线','','路线 (*.json)')
        if path:
            try:
                mid,rid,pts=load_route(path)
                if mid!=self.grid.map_id:
                    raise ValueError('路线属于地图 %s，请先加载对应地图；不会自动套用坐标'%mid)
                validate_points(pts,self.grid)
                self.points=pts
                self.route_name.setText(rid)
                self.dirty=False
                self.sync_table()
                self.log('已加载路线：'+rid)
            except Exception as exc: self.error(exc)

    def export_requests(self):
        try:
            validate_points(self.points,self.grid)
            rid=self.route_name.text().strip()
            if not rid: raise ValueError('路线 ID 不能为空')
            data={'note':'请求预览；未发送。仅在地图 ID 与实机一致且控制权明确时使用。',
                  'save_route':request('add_nav_path',path_payload(self.grid.map_id,rid,self.points),1),
                  'start_route':request('start_multi_nav',[self.grid.map_id,rid],2),
                  'start_by_points':request('start_multi_nav_by_points',[self.grid.map_id,[p.pose() for p in self.points]],3)}
            path,_=QFileDialog.getSaveFileName(self,'导出协议请求','requests.json','JSON (*.json)')
            if path:
                atomic_json(path,data)
                self.log('已导出请求预览，未发送：'+path)
        except Exception as exc: self.error(exc)

    def switch_mode(self,index):
        live=index==1
        if live==self.live: return
        if self.stack_locked() and not live:
            self.mode.blockSignals(True)
            self.mode.setCurrentIndex(1)
            self.mode.blockSignals(False)
            self.error('请先处理系统启动流程，再切回离线演示')
            return
        if self.live and self.states['nav'] in ('Active','Pause'):
            self.error('请先停止真实导航，再切换模式')
            self.mode.blockSignals(True)
            self.mode.setCurrentIndex(1)
            self.mode.blockSignals(False)
            return
        if not self.discard_ok():
            self.mode.blockSignals(True)
            self.mode.setCurrentIndex(1 if self.live else 0)
            self.mode.blockSignals(False)
            return
        self.disconnect_device()
        self.stop_ros()
        self.live=live
        self.pad.stop()
        self.write_enabled.setChecked(False)
        self.nav_owner.setChecked(False)
        self.live_map.setChecked(False)
        self.ros_map.setChecked(False)
        self.loaded_map_id=None
        self.ros_status={}
        self.ros_note.setText('地图/位姿/激光：等待订阅')
        self.status_times={}
        self.states={'mapping':'Unknown','loc':'Unknown','nav':'Unknown'} if live else {'mapping':'MappingReady','loc':'ContinuousLoc','nav':'StandBy'}
        self.map_choice.blockSignals(True)
        self.map_choice.clear()
        if live: self.map_choice.addItem('尚未读取设备地图',None)
        else: self.map_choice.addItems(['演示办公室','已有实机地图（本地）'])
        self.map_choice.blockSignals(False)
        if live:
            self.install_map(GridMap('等待设备地图',100,80,.1,(0,0,0),[-1]*8000,source='empty'))
        else:
            self.install_map(demo_map())
        self.canvas.robot=None
        self.scan_view.points=[]
        self.scan_view.caption='等待实机激光' if live else '模拟激光'
        self.scan_view.update()
        if not live:
            self.robot=(-3.8,-2.6,0)
            self.load_example()
        self.log('进入'+('实机模式：默认只读，尚未连接。' if live else '离线演示模式。'))
        self.refresh()

    def toggle_connection(self):
        if self.worker: self.disconnect_device(); return
        self.stack_state=self.stack.status()
        if self.stack_locked():
            self.error('start_all / 自定义导航正在占用设备连接；请在系统启动页处理')
            return
        try:
            worker=WsWorker(self.url.text().strip(),self)
            self.worker=worker
            worker.result.connect(lambda seq,func,data,w=worker:self.on_result(seq,func,data) if self.worker is w else None)
            worker.error.connect(lambda seq,func,err,w=worker:self.on_error(seq,func,err) if self.worker is w else None)
            worker.event.connect(lambda msg,w=worker:self.log('设备通知：'+json.dumps(msg,ensure_ascii=False)[:800]) if self.worker is w else None)
            worker.start()
            self.log('正在连接 '+self.url.text().strip()+'；持续连接、串行请求，无自动运动。')
            self.poll()
            self.map_read_complete=False
            self.map_retry_delay=2.
            self.request_maps()
        except Exception as exc: self.error(exc)
        self.refresh()

    def request_maps(self):
        if not self.live or not self.worker:return
        if any(f=='get_all_pgm_map' for f,v in self.pending.values()):return
        self.map_retry_at=time.monotonic()+self.map_retry_delay
        self.send('get_all_pgm_map',priority=20)

    def disconnect_device(self):
        if self.worker:
            worker,self.worker=self.worker,None
            worker.shutdown()
            self.retired.append(worker)
            self.log('已停止设备通信；断开界面不等于机器人任务已停止。')
        self.pending={}
        self.online=False
        self.loaded_map_id=None
        self.known_paths={}
        self.write_enabled.setChecked(False)
        self.nav_owner.setChecked(False)
        self.refresh()

    def send(self,func,value=None,priority=10):
        if not func.startswith('get_'):
            self.stack_state=self.stack.status()
        if self.stack_locked():
            self.error('start_all 接管期间不发送官方 WebSocket 请求')
            return None
        if not self.live or not self.worker:
            self.error('请先切换实机模式并连接设备'); return None
        read=func.startswith('get_')
        if not read and func!='stop_nav' and not self.write_enabled.isChecked():
            self.error('真实操作未启用'); return None
        if not read and any(f==func for f,v in self.pending.values()):
            self.error('此操作仍在等待响应，请勿重复提交'); return None
        if not read and func!='stop_nav' and any(not f.startswith('get_') for f,v in self.pending.values()):
            self.error('上一项设备操作仍在等待响应，请稍后再试'); return None
        try:
            seq=self.worker.submit(func,value,priority)
            if seq is not None:
                self.pending[seq]=(func,value)
                if not read: self.log('发送请求 #%d：%s（等待设备确认）'%(seq,func))
            self.refresh()
            return seq
        except Exception as exc: self.error(exc); return None

    def poll(self):
        if self.live and self.worker and not self.stack_locked():
            if not self.map_read_complete and time.monotonic()>=self.map_retry_at:
                self.request_maps()
            for func in ('get_loc_status','get_mapping_status','get_nav_status'):
                self.send(func,priority=20)

    def refresh_live_map(self):
        if self.live and self.worker and not self.stack_locked() and self.live_map.isChecked():
            self.send('get_pgm_map',priority=25)

    def on_result(self,seq,func,data):
        if seq not in self.pending: return  # cancelled/retired response cannot update the UI
        _,value=self.pending.pop(seq,(func,None))
        self.online=True
        self.last_response=time.monotonic()
        try:
            state_key={'get_loc_status':'loc','get_mapping_status':'mapping','get_nav_status':'nav'}.get(func)
            if state_key:
                old=self.states[state_key]
                self.states[state_key]=str(data)
                self.status_times[state_key]=time.monotonic()
                if old!=str(data): self.log('%s 状态：%s → %s'%(state_key,old,data))
                if state_key=='loc' and data in ('Error','LocLost') and old!=str(data):
                    # Only stop a mission this console has explicitly been allowed to control.
                    if self.nav_owner.isChecked(): self.stop_task()
                    self.log('定位异常：停止自动任务；实机键盘通道尚未部署，请用现有遥控器接管。')
                if state_key=='mapping' and data=='MappingSaveEnd': self.log('设备报告：地图保存完成。可重新读取地图列表。')
            elif func=='get_all_pgm_map':
                if not isinstance(data,dict): raise ValueError('地图列表响应结构不符，请导出诊断记录')
                maps={}
                for mid,entry in data.items():
                    p=entry[0] if isinstance(entry,list) and entry else entry
                    if isinstance(p,dict) and 'info' in p and 'data' in p: maps[str(mid)]=p
                self.raw_maps=maps
                self.map_read_complete=True
                self.map_retry_delay=2.
                self.map_choice.blockSignals(True)
                self.map_choice.clear()
                for mid in maps: self.map_choice.addItem(mid,mid)
                if not maps: self.map_choice.addItem('设备未返回地图',None)
                idx=self.map_choice.findData(self.grid.map_id)
                self.map_choice.setCurrentIndex(idx if idx>=0 else 0)
                self.map_choice.blockSignals(False)
                self.log('已读取 %d 张设备地图。'%len(maps))
                if maps and self.grid.source in ('empty','demo') and not self.dirty: self.choose_map(0)
            elif func=='get_pgm_map':
                grid=GridMap.from_payload('__current__',data,self.map_encoding())
                # Current-map API does not identify which map was returned. Display without rebinding routes.
                if not self.points and not self.dirty:
                    self.grid=grid
                    self.canvas.set_grid(grid,fit=False)
                else: self.log('已收到当前栅格；为保留路线绑定未替换编辑地图。请清空路线后刷新。')
            elif func=='loc_load_map':
                self.loaded_map_id=value
                self.states['loc']='MapLoading'
                self.status_times.pop('loc',None)
                self.log('加载地图请求已接受；等待 ContinuousLoc，尚不能认定定位成功。')
                self.poll()
            elif func=='reset_loc':
                self.loaded_map_id=None
                self.states['loc']='InitLocalization'
                self.status_times.pop('loc',None)
                self.log('重置定位请求已接受。')
                self.poll()
            elif func=='stop_mapping':
                self.states['mapping']='MappingSaveBegin'
                self.log('停止建图请求已接受，正在等待地图保存完成；请保持状态轮询。')
                self.poll()
            elif func=='start_mapping':
                self.states['mapping']='InitWaitSensor'
                self.loaded_map_id=None
                self.log('开始建图请求已接受，等待设备运行状态。')
                self.poll()
            elif func=='get_all_paths_by_mapid':
                self.receive_paths(value,data)
            elif func in ('add_nav_path','modify_nav_path'):
                mid=value[0]
                rid=value[1] if func=='add_nav_path' else value[2]
                self.known_paths.setdefault(mid,set()).add(rid)
                self.log('设备确认路线已保存：'+rid)
            else:
                self.log('设备已接受 %s；执行结果以状态反馈为准。'%func)
                if func in ('start_multi_nav_by_points','pause_nav','continue_nav','stop_nav'):
                    self.states['nav']='等待设备状态'
                    self.status_times.pop('nav',None)
                    self.poll()
        except Exception as exc: self.error('%s：%s'%(func,exc))
        self.refresh()

    def on_error(self,seq,func,error):
        self.pending.pop(seq,None)
        self.log('请求 #%d %s 失败：%s'%(seq,func,error))
        state_key={'get_loc_status':'loc','get_mapping_status':'mapping','get_nav_status':'nav'}.get(func)
        if state_key:
            self.status_times.pop(state_key,None)
            self.write_enabled.setChecked(False)
            self.nav_owner.setChecked(False)
        if func=='connection':
            self.online=False
            self.write_enabled.setChecked(False)
            self.nav_owner.setChecked(False)
        if func=='get_all_pgm_map':
            self.map_retry_at=time.monotonic()+self.map_retry_delay
            self.map_retry_delay=min(30.,self.map_retry_delay*2)
            self.log('地图读取失败，将自动重试只读请求；也可点击“读取 / 重试设备地图”。')
        if not func.startswith('get_') and func!='connection':
            self.log('未自动重试操作；响应丢失时设备可能已执行，请先核查状态。')
        self.refresh()

    def upload_route(self):
        try:
            self.validate_live_route()
            rid=self.route_name.text().strip()
            if not rid: raise ValueError('路线 ID 不能为空')
            payload=path_payload(self.grid.map_id,rid,self.points)
            if rid in self.known_paths.get(self.grid.map_id,set()):
                self.send('modify_nav_path',[self.grid.map_id,rid,rid,payload[2]])
            else:
                self.send('add_nav_path',payload)
        except Exception as exc: self.error(exc)

    def receive_paths(self,map_id,data):
        if map_id!=self.grid.map_id: raise ValueError('当前地图已改变，忽略旧地图路线响应')
        if not isinstance(data,list) or len(data)<3 or data[0]!=map_id or not isinstance(data[2],dict):
            raise ValueError('路线列表响应格式不匹配')
        paths=data[2]
        self.known_paths[map_id]=set(paths)
        if not paths: self.log('该地图还没有保存路线'); return
        name,ok=QInputDialog.getItem(self,'选择设备路线','路线 ID',list(paths),0,False)
        if ok and self.discard_ok():
            points=[]
            for pname,pose in paths[name]:
                points.append(Waypoint(str(pname),finite(pose['position']['x']),finite(pose['position']['y']),yaw_of(pose['orientation'])))
            validate_points(points,self.grid)
            self.points=points
            self.route_name.setText(name)
            self.dirty=False
            self.sync_table()
            self.log('已读取设备路线：'+name)

    def validate_live_route(self):
        if not self.live or self.grid.source!='robot' or self.grid.map_id=='__current__' or self.grid.frame!='map':
            raise ValueError('请先从设备地图列表加载具体地图，不能下发演示/本地/匿名栅格路线')
        if self.grid.map_id!=self.map_choice.currentData():
            raise ValueError('所选地图与当前画布不一致，请重新选择')
        validate_points(self.points,self.grid)

    def load_localization(self):
        if not self.live:
            self.states['loc']='ContinuousLoc'
            self.demo_mode='待命'
            self.log('模拟：定位恢复，可重新启动巡航。')
            self.refresh(); return
        if self.grid.source!='robot' or self.grid.map_id=='__current__':
            self.error('请先选择一张有明确 ID 的设备地图'); return
        if self.states['nav'] in ('Active','Pause'):
            self.error('请先停止导航任务'); return
        self.send('loc_load_map',self.grid.map_id)

    def reset_localization(self):
        if not self.live:
            self.inject_loc_failure(); return
        if self.states['nav'] in ('Active','Pause'):
            self.error('请先停止导航任务'); return
        self.send('reset_loc')

    def start_mapping(self):
        if self.live:
            if self.states['nav'] not in ('StandBy','Cancelled','Succeed','Failed'):
                self.error('请确认导航任务已停止且状态已刷新'); return
            if self.states['mapping'] in ('MappingRunning','MappingSaveBegin'):
                self.error('建图运行或保存中，请等待'); return
            self.send('start_mapping',0)
        else:
            self.stop_demo()
            self.states['mapping']='MappingRunning'
            self.log('模拟：建图开始。此地图为演示数据，不代表真实采集。')
            self.refresh()

    def stop_mapping(self):
        if self.live:
            self.send('stop_mapping',0,priority=2)
        else:
            self.states['mapping']='MappingSaveBegin'
            self.log('模拟：建图结束，正在保存。')
            QTimer.singleShot(1200,self.demo_map_saved)
            self.refresh()

    def demo_map_saved(self):
        if not self.live and self.states['mapping']=='MappingSaveBegin':
            self.states['mapping']='MappingSaveEnd'
            self.log('模拟：地图保存流程完成。真实文件可通过“导出当前地图”保存。')
            self.refresh()

    def start_nav(self):
        try:
            validate_points(self.points,self.grid)
            if self.live:
                self.validate_live_route()
                if not self.nav_owner.isChecked(): raise ValueError('请先停用其他导航/守护脚本，并启用官方导航控制')
                if self.loaded_map_id!=self.grid.map_id: raise ValueError('请先通过本界面加载所选定位地图')
                if self.states['loc']!='ContinuousLoc' or time.monotonic()-self.status_times.get('loc',0)>10:
                    raise ValueError('需要新鲜的 ContinuousLoc 定位状态')
                if self.states['nav']!='StandBy' or time.monotonic()-self.status_times.get('nav',0)>10:
                    raise ValueError('需要新鲜的 StandBy 导航状态')
                if self.states['mapping'] in ('MappingRunning','MappingSaveBegin'): raise ValueError('建图/保存期间不能启动巡航')
                self.send('start_multi_nav_by_points',[self.grid.map_id,[p.pose() for p in self.points]])
            else:
                if self.grid.source!='demo': raise ValueError('离线模拟导航仅在演示办公室地图运行；已有实机地图可编辑路线')
                if self.states['loc']!='ContinuousLoc': raise ValueError('模拟定位无效，请先点击“加载所选地图定位”恢复')
                if self.states['mapping'] in ('MappingRunning','MappingSaveBegin'): raise ValueError('请先结束建图并等待保存')
                self.pad.stop()
                self.demo_target=0
                self.demo_mode='自动巡航'
                self.states['nav']='Active'
                self.plan_next()
                self.log('模拟巡航开始：%d 个点，含障碍膨胀的本地 A* 演示。'%len(self.points))
        except Exception as exc: self.error(exc)
        self.refresh()

    def pause_nav(self):
        if self.live:
            if self.states['nav'] not in ('Active','Pause'): return
            if self.states['nav']=='Pause' and (self.states['loc']!='ContinuousLoc' or time.monotonic()-self.status_times.get('loc',0)>10):
                self.error('定位状态无效或过期，不能继续导航'); return
            self.send('pause_nav' if self.states['nav']=='Active' else 'continue_nav')
        elif self.demo_mode=='自动巡航':
            self.demo_mode='暂停'; self.states['nav']='Pause'
        elif self.demo_mode=='暂停':
            self.demo_mode='自动巡航'; self.states['nav']='Active'
        self.refresh()

    def stop_demo(self):
        self.demo_mode='待命'
        self.demo_target=0
        self.demo_cmd=(0,0)
        self.demo_path=[]
        self.canvas.path=[]
        self.states['nav']='Cancelled'
        self.pad.stop()

    def stop_task(self):
        self.stack_state=self.stack.status()
        if self.stack_locked():
            self.stop_stack()
            return
        if self.live:
            if self.worker:
                self.worker.cancel_pending()
                self.pending={}
                if self.send('stop_nav',priority=0) is not None:
                    self.log('已请求停止官方导航；未确认前不能视为已停车。旧 custom_nav/物理急停请单独处理。')
            else: self.error('设备未连接，无法发送停止请求；请使用机器狗遥控器/物理急停')
        else:
            self.stop_demo()
            self.log('模拟任务已停止。')
        self.refresh()

    def manual_takeover(self):
        if self.live:
            self.stop_task()
            self.log('实机速度仲裁尚未部署；本界面不会发布 /cmd_vel，请使用现有遥控器。')
        else:
            self.stop_demo()
            self.demo_mode='手动接管'
            self.pad.setFocus()
            self.log('模拟手动接管已启用：按住方向键运动，松开或失焦停止。')
        self.refresh()

    def teleop(self,v,w):
        if not self.live and self.demo_mode=='手动接管': self.demo_cmd=(v,w)
        else: self.demo_cmd=(0,0)

    def inject_loc_failure(self):
        if self.live: return
        self.stop_demo()
        self.states['loc']='LocLost'
        self.demo_mode='手动接管'
        self.pad.setFocus()
        self.log('模拟定位丢失：自动任务已停止，切到手动待接管；需按住键才会移动。')
        self.refresh()

    def inject_obstacle(self):
        if self.live or self.grid.source!='demo': return
        x,y,a=self.robot
        ox,oy=x+.9*math.cos(a),y+.9*math.sin(a)
        for gy in range(self.grid.height):
            for gx in range(self.grid.width):
                wx,wy=self.grid.cell_to_world(gx+.5,gy+.5)
                if abs(wx-ox)<.20 and abs(wy-oy)<.20: self.grid.cells[gy*self.grid.width+gx]=100
        self.canvas.set_grid(self.grid,fit=False)
        self.log('模拟：前方新增障碍。自动巡航将重规划；手动控制受近障停车保护。')
        if self.demo_mode=='自动巡航': self.plan_next()

    def load_example(self):
        self.points=[Waypoint('入口',-3.7,-2.6,0),Waypoint('走廊',-.3,-2.5,math.pi/2),
                     Waypoint('巡检区',3.6,1.7,math.pi),Waypoint('返回点',-3.6,2.6,-math.pi/2)]
        self.dirty=False
        self.sync_table(0)

    def preview_demo_route(self):
        if self.live or self.grid.source!='demo' or not self.editing_allowed(): return
        try:
            validate_points(self.points,self.grid)
            path=[]
            start=self.robot[:2]
            for point in self.points:
                leg=demo_plan(self.grid,start,(point.x,point.y))
                if not leg: raise ValueError('无法到达 '+point.name+'，请调整点位或障碍')
                path.extend(leg if not path else leg[1:])
                start=(point.x,point.y)
            self.canvas.path=path
            self.canvas.viewport().update()
            self.log('模拟规划预览：%d 个巡航点 · A* + 可通行直线简化 + 安全圆角；未启动运动。'%len(self.points))
        except Exception as exc:
            self.canvas.path=[]
            self.error(exc)

    def reset_demo(self):
        if self.live or not self.editing_allowed() or not self.discard_ok(): return
        self.install_map(demo_map())
        self.robot=(-3.8,-2.6,0)
        self.states={'mapping':'MappingReady','loc':'ContinuousLoc','nav':'StandBy'}
        self.load_example()
        self.restore_map_choice()

    def plan_next(self):
        if self.demo_target>=len(self.points):
            self.demo_mode='已完成'
            self.states['nav']='Succeed'
            self.demo_path=[]
            self.canvas.path=[]
            self.log('模拟巡航完成：已到达全部巡航点。')
            return
        goal=self.points[self.demo_target]
        self.demo_path=demo_plan(self.grid,self.robot[:2],(goal.x,goal.y))
        self.canvas.path=self.demo_path[:]
        if not self.demo_path:
            self.demo_mode='无可行路径'
            self.states['nav']='Failed'
            self.log('模拟规划失败：机器人或目标过于靠近障碍，或路径被阻断；已停止。')

    def collision(self,x,y):
        return not segment_clear(self.grid,(x,y),(x,y),ROBOT_RADIUS)

    def simulate_scan(self):
        x,y,a=self.robot
        points=[]
        for i in range(72):
            angle=i*math.tau/72
            for j in range(1,61):
                r=j*.075
                if not self.grid.valid_point(x+r*math.cos(a+angle),y+r*math.sin(a+angle)):
                    points.append((r*math.cos(angle),r*math.sin(angle)))
                    break
        return points

    def tick(self):
        if self._closed: return
        now=time.monotonic()
        if self.live:
            if self.online and now-self.last_response>12:
                self.online=False
                self.write_enabled.setChecked(False)
                self.nav_owner.setChecked(False)
                self.log('设备状态已过期：真实操作已锁定。连接丢失不能保证机器人已停止。')
            if now-self.last_pose>1.5 or self.states['loc'] in ('LocLost','Error'): self.canvas.robot=None
            if now-self.last_path>1.5: self.canvas.path=[]
            if now-self.last_scan>1.0:
                self.canvas.scan=[]
                self.scan_view.points=[]
                self.scan_view.caption='激光数据缺失 / 已过期'
                self.scan_view.update()
        elif self.grid.source=='demo':
            x,y,a=self.robot
            if self.demo_mode=='自动巡航' and self.demo_path:
                tx,ty=self.demo_path[0]
                d=math.hypot(tx-x,ty-y)
                if d>1e-10: a=math.atan2(ty-y,tx-x)
                reached=d<=.06+1e-10
                nx,ny=(tx,ty) if reached else (x+.06*math.cos(a),y+.06*math.sin(a))
                if segment_clear(self.grid,(x,y),(nx,ny),ROBOT_RADIUS):
                    x,y=nx,ny
                    if reached:
                        self.demo_path.pop(0)
                        if not self.demo_path:
                            a=self.points[self.demo_target].yaw
                            self.log('模拟到达：'+self.points[self.demo_target].name)
                            self.demo_target+=1
                            self.robot=(x,y,a)
                            self.plan_next()
                else:
                    self.demo_mode='近障停车'
                    self.states['nav']='Failed'
                    self.log('模拟近障保护触发，任务已停止。')
            elif self.demo_mode=='手动接管':
                v,w=self.demo_cmd
                a+=w*.1
                nx,ny=x+v*.1*math.cos(a),y+v*.1*math.sin(a)
                if segment_clear(self.grid,(x,y),(nx,ny),ROBOT_RADIUS):
                    x,y=nx,ny
                    self.demo_block=False
                elif v and not self.demo_block:
                    self.demo_block=True
                    self.log('模拟近障保护：平移已拦截，可以转向或后退。')
            self.robot=(x,y,a)
            self.canvas.robot=self.robot
            scan=self.simulate_scan()
            self.canvas.scan=scan
            self.scan_view.points=scan
            self.scan_view.caption='模拟激光 · 不代表实机避障已验收'
            self.scan_view.update()
        else:
            self.canvas.robot=None
            self.canvas.scan=[]
            self.scan_view.points=[]
            self.scan_view.caption='本地地图编辑 · 无实时激光'
            self.scan_view.update()
        self.canvas.viewport().update()
        self.refresh()

    def refresh(self,*args):
        if not hasattr(self,'metrics') or not hasattr(self,'start_nav_btn'): return
        live=self.live
        ready=live and self.online and self.worker is not None
        writable=ready and self.write_enabled.isChecked()
        simulated=not live and self.grid.source=='demo'
        live_source='本地历史地图参考' if self.grid.source=='local' else '等待设备地图' if self.grid.source=='empty' else '设备地图 / ROS 数据'
        self.banner.setText(('●  实机模式  /  '+('操作已解锁' if writable else '只读监测')+'   ·   '+live_source+'；真实键盘控制尚未部署') if live else
                            ('◈  离线演示  /  DEMO   ·   运动、激光与任务状态为模拟；地图编辑和路线保存可直接使用' if simulated else
                             '◈  本地地图  /  OFFLINE   ·   可编辑与保存路线；未连接机器人，无实时定位和激光'))
        self.metrics['connection'].setText('通信连接\n'+('持续连接' if ready else ('等待响应' if self.worker else '未连接')) if live else '通信连接\n离线演示' if simulated else '通信连接\n本地编辑')
        self.metrics['connection'].set_detail('WebSocket · ROS 2' if live else '本地模拟器 · 无设备通信' if simulated else '本地地图文件', 'good' if ready else 'neutral')
        translated={'ContinuousLoc':'定位正常','LocLost':'定位丢失','MappingReady':'建图就绪','MappingRunning':'建图运行中',
                    'MappingSaveBegin':'正在保存地图','MappingSaveEnd':'地图保存完成','StandBy':'导航就绪','Active':'导航运行中',
                    'Pause':'已暂停','Succeed':'任务成功','Failed':'任务失败','Cancelled':'任务已取消','Unknown':'未知',
                    'Init':'定位功能初始化','MapLoading':'地图加载中','InitLocalization':'定位初始化','InitWaitSensor':'等待传感器'}
        for key,title in [('loc','定位'),('mapping','建图'),('nav','任务')]:
            val=self.states[key]
            stale=live and time.monotonic()-self.status_times.get(key,0)>12
            display=translated.get(val,val)+(' · 待刷新' if stale else '')
            if not live and key=='nav': display=self.demo_mode
            if not live and not simulated: display='无实时数据' if key!='nav' else '路线编辑'
            if stale: display=translated.get(val,val)+' · 待刷新'
            self.metrics[key].setText(title+'\n'+display)
            tone='warn' if stale or val=='Unknown' else ('bad' if val in ('LocLost','Failed','Error') else 'good')
            detail=('状态超时，请检查连接' if stale else val) if live else ('模拟状态 / DEMO' if simulated else '离线文件，不代表设备状态')
            if live and not self.status_times.get(key):
                self.metrics[key].setText(title+'\n尚未读取')
                detail='等待设备状态' if self.worker else '未连接设备'
                tone='neutral'
            self.metrics[key].set_detail(detail, tone if live or simulated else 'neutral')
        self.connect_btn.setEnabled(live)
        self.connect_btn.setText('断开设备通信' if self.worker else '连接设备')
        self.url.setEnabled(live and not self.worker)
        self.write_enabled.setEnabled(ready)
        map_pending=any(f=='get_all_pgm_map' for f,v in self.pending.values())
        self.fetch_btn.setEnabled(live and bool(self.worker) and not map_pending)
        self.fetch_btn.setText('正在读取设备地图…' if map_pending else '读取 / 重试设备地图')
        self.live_map.setEnabled(live and bool(self.worker))
        self.encoding.setEnabled(live)
        self.ros_button.setEnabled(live)
        for widget in (self.url,self.connect_btn,self.write_enabled,self.ros_button,self.fetch_btn,
                       self.encoding,self.live_map,self.ros_map,self.upload_btn,self.read_routes_btn,self.nav_owner):
            widget.setVisible(live)
        if not live: self.ros_note.setText('本地模拟器已就绪\n无需网络或机器狗')
        self.nav_owner.setEnabled(writable)
        self.ros_map.setEnabled(live and self.monitor is not None)
        busy={f for f,v in self.pending.values()}
        for btn,func in [(self.start_map_btn,'start_mapping'),(self.stop_map_btn,'stop_mapping'),
                         (self.load_loc_btn,'loc_load_map'),(self.reset_loc_btn,'reset_loc')]:
            btn.setEnabled((simulated or writable) and func not in busy)
        self.upload_btn.setEnabled(writable and bool(self.points) and 'add_nav_path' not in busy)
        self.read_routes_btn.setEnabled(ready and self.grid.source=='robot' and self.grid.map_id!='__current__')
        self.start_nav_btn.setEnabled(bool(self.points) and (simulated or (writable and self.nav_owner.isChecked())) and 'start_multi_nav_by_points' not in busy)
        self.pause_btn.setEnabled((not live or (writable and self.nav_owner.isChecked())) and self.states['nav'] in ('Active','Pause'))
        self.pad.setEnabled(simulated)
        self.manual_btn.setEnabled(simulated or bool(self.worker))
        for btn in (self.fail_btn,self.obstacle_btn): btn.setEnabled(simulated)
        self.example_btn.setEnabled(not live)
        self.map_meta.setText('等待设备地图 · 可在“设置”页重试读取' if self.grid.source=='empty' else '%s  /  %s  /  %.2f m 每格' % (self.grid.map_id,self.grid.frame,self.grid.resolution))
        self.map_meta.setToolTip(self.map_meta.text())
        source={'demo':'演示地图','robot':'设备地图','local':'本地地图','empty':'尚未加载'}.get(self.grid.source,'只读地图')
        self.map_details.setText('%s\n%d × %d 栅格  ·  %.2f m / 格'%(source,self.grid.width,self.grid.height,self.grid.resolution))
        self.clock_label.setText(datetime.now().strftime('%m.%d  %H:%M:%S'))
        count=len(self.points)
        self.progress.setVisible(simulated)
        self.progress.setRange(0,max(1,count))
        completed=min(self.demo_target,count) if self.demo_mode in ('自动巡航','暂停','已完成','近障停车','无可行路径') else 0
        self.progress.setValue(completed)
        suffix='已到达 %d / %d'%(completed,count) if simulated and self.demo_mode in ('自动巡航','暂停','已完成') else '按序执行' if count else '在地图上添加巡航点'
        self.route_summary.setText('%02d 个巡航点  ·  %s'%(count,suffix))
        self.pause_btn.setText('继续' if self.states['nav']=='Pause' else '暂停')
        # Mutating waypoints mid-mission changes the goal list; lock editing until stopped.
        editing=self.demo_mode not in ('自动巡航','暂停') and not (live and self.states['nav'] in ('Active','Pause'))
        self.table.setEditTriggers(QAbstractItemView.DoubleClicked|QAbstractItemView.EditKeyPressed if editing else QAbstractItemView.NoEditTriggers)
        self.add_btn.setEnabled(editing)
        self.preview_btn.setEnabled(simulated and editing and bool(self.points))
        self.reset_scene_btn.setEnabled(not live and editing)
        self.route_name.setReadOnly(not editing)
        if self.demo_mode in ('自动巡航','暂停') or (live and self.states['nav'] in ('Active','Pause')):
            self.start_nav_btn.setEnabled(False)
        if not editing and self.add_btn.isChecked(): self.add_btn.setChecked(False)
        self.refresh_stack()

    def editing_allowed(self):
        if self.demo_mode in ('自动巡航','暂停') or (self.live and self.states['nav'] in ('Active','Pause')):
            self.error('请先停止当前巡航，再修改路线或地图')
            return False
        return True

    def toggle_ros(self):
        if self.monitor: self.stop_ros(); return
        if self._stack_pending:
            self.error('启动交接中，请等待通信线程退出')
            return
        if self.stack_locked() and (os.environ.get('ROS_DOMAIN_ID')!='24' or os.environ.get('RMW_IMPLEMENTATION')!='rmw_zenoh_cpp'):
            self.error('当前窗口使用离线 ROS 环境；请关闭后用 start_console.sh --stack 打开只读监测')
            return
        mon=RosMonitor(self)
        self.monitor=mon
        mon.telemetry.connect(lambda kind,data,m=mon:self.ros_data(kind,data) if self.monitor is m else None)
        mon.error.connect(lambda text:self.error('ROS 监测：'+text))
        mon.start()
        self.ros_button.setText('停止 ROS 只读监测')
        self.log('启动 ROS 只读监测，domain=%s，RMW=%s。'%(os.environ.get('ROS_DOMAIN_ID','0'),os.environ.get('RMW_IMPLEMENTATION','默认')))
        self.refresh()

    def stop_ros(self):
        if self.monitor:
            mon,self.monitor=self.monitor,None
            mon.shutdown()
            self.retired.append(mon)
        if self.live:
            self.canvas.path=[]
            self.canvas.robot=None
            self.canvas.scan=[]
            self.scan_view.points=[]
            self.last_pose=self.last_scan=self.last_path=self.last_custom_nav=self.last_odom_source=0
        if hasattr(self,'ros_button'): self.ros_button.setText('启动 ROS 只读监测')

    def ros_data(self,kind,data):
        now=time.monotonic()
        if kind!='ready': self.ros_status['ready']='ROS 已收到设备数据（只读）'
        if kind=='pose':
            self.last_pose=now
            self.canvas.robot=tuple(data[:3]) if self.grid.source!='empty' else None
            self.ros_status['pose']='map 位姿：(%.2f, %.2f)'%(data[0],data[1])
        elif kind=='scan':
            self.last_scan=now
            self.canvas.scan=data if self.grid.source!='empty' else []
            self.scan_view.points=data
            self.ros_status['scan']='激光：%d 个有效点'%len(data)
            self.scan_view.caption='实机激光 · 仅显示，不执行避障'
            self.scan_view.update()
        elif kind=='map' and self.ros_map.isChecked():
            if not self.points and not self.dirty:
                self.grid=data
                self.canvas.set_grid(data,fit=False)
        elif kind=='path':
            self.last_path=now
            self.canvas.path=list(data) if self.grid.source=='ros' and self.grid.frame=='map' and self.ros_map.isChecked() else []
        elif kind=='path_unavailable':
            self.canvas.path=[]
            self.last_path=0
        elif kind in ('ready','pose_unavailable','scan_unavailable','odom_type'):
            key={'pose_unavailable':'pose','scan_unavailable':'scan'}.get(kind,kind)
            self.ros_status[key]=('位姿源：' if kind=='odom_type' else '')+str(data)
            if kind=='pose_unavailable': self.canvas.robot=None
            if kind=='odom_type': self.last_odom_source=now
        elif kind=='custom_nav':
            self.last_custom_nav=now
            self.nav_owner.setChecked(False)
            self.ros_status['custom_nav']='custom_nav：'+str(data)
        self.ros_note.setText('\n'.join(self.ros_status[k] for k in ('ready','odom_type','pose','scan','custom_nav') if k in self.ros_status))

    def export_log(self):
        path,_=QFileDialog.getSaveFileName(self,'导出诊断记录','robot_console_diagnostics.json','JSON (*.json)')
        if path:
            try:
                atomic_json(path,{'version':'0.1.0','mode':'robot' if self.live else 'demo',
                  'time':datetime.now().isoformat(),'states':self.states,'map_id':self.grid.map_id,
                  'loaded_localization_map':self.loaded_map_id,'route':[p.record() for p in self.points],
                  'ros_domain_id':os.environ.get('ROS_DOMAIN_ID'),'rmw':os.environ.get('RMW_IMPLEMENTATION'),
                  'stack':self.stack_state,'stack_script':str(self.stack.script),'stack_log':self.stack.log_tail(),
                  'log':self.log_lines})
                self.log('诊断记录已导出：'+path)
            except Exception as exc: self.error(exc)

    def eventFilter(self,obj,event):
        if event.type()==QEvent.ApplicationDeactivate and hasattr(self,'pad'):
            self.pad.stop()
        return super().eventFilter(obj,event)

    def shutdown(self):
        if self._closed: return
        self._closed=True
        self.timer.stop()
        self.poll_timer.stop()
        self.map_timer.stop()
        self.stack_timer.stop()
        self._stack_pending=False
        if self._stack_check:
            self._stack_check.wait(6000)
        self.pad.stop()
        self.disconnect_device()
        self.stop_ros()
        for thread in self.retired:
            thread.shutdown()
            thread.wait(7000)
        QApplication.instance().removeEventFilter(self)

    def closeEvent(self,event):
        if not self.discard_ok(): event.ignore(); return
        self.shutdown()
        event.accept()


def main():
    app=QApplication(sys.argv)
    app.setApplicationName('智航机器狗控制台')
    win=Console()
    win.show()
    code=app.exec_()
    win.shutdown()
    return code


if __name__=='__main__':
    sys.exit(main())
