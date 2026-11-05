import math
from PyQt5.QtCore import Qt, QPointF, QRectF, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen, QBrush, QPolygonF, QFont
from PyQt5.QtWidgets import QGraphicsView, QGraphicsScene, QWidget


class MapCanvas(QGraphicsView):
    pointAdded = pyqtSignal(float, float, float)
    pointSelected = pyqtSignal(int)
    pointMoved = pyqtSignal(int, float, float)
    cursorWorld = pyqtSignal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('mapCanvas')
        self.setScene(QGraphicsScene(self))
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setBackgroundBrush(QColor('#111e2d'))
        self.setFrameShape(QGraphicsView.NoFrame)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setMouseTracking(True)
        self.grid = None
        self.points = []
        self.selected = -1
        self.robot = None
        self.scan = []
        self.path = []
        self.add_mode = False
        self._press = None
        self._moving = -1
        self._preview = None

    def set_grid(self, grid, fit=True):
        from PyQt5.QtGui import QPixmap
        self.grid = grid
        self.scene().clear()
        if grid.source!='empty':
            self.scene().addPixmap(QPixmap.fromImage(grid.image()))
        self.scene().setSceneRect(QRectF(-12,-12,grid.width+24,grid.height+24))
        if fit:
            self.fit_map()
        self.viewport().update()

    def fit_map(self):
        if self.grid:
            self.fitInView(self.sceneRect(), Qt.KeepAspectRatio)

    def set_add_mode(self, enabled):
        self.add_mode = enabled
        self.setDragMode(QGraphicsView.NoDrag if enabled else QGraphicsView.ScrollHandDrag)
        self.setCursor(Qt.CrossCursor if enabled else Qt.OpenHandCursor)

    def scene_world(self, pos):
        p = self.mapToScene(pos)
        return self.grid.pixel_to_world(p.x(),p.y())

    def hit_point(self, pos):
        if not self.grid:
            return -1
        for i,p in enumerate(self.points):
            q = self.mapFromScene(QPointF(*self.grid.world_to_pixel(p.x,p.y)))
            if math.hypot(q.x()-pos.x(),q.y()-pos.y()) < 16:
                return i
        return -1

    def mousePressEvent(self, event):
        if self.grid and event.button() == Qt.LeftButton:
            if self.add_mode:
                self._press = self.scene_world(event.pos())
                self._preview = self._press
                event.accept()
                return
            hit = self.hit_point(event.pos())
            if hit >= 0:
                self._moving = hit
                self.pointSelected.emit(hit)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.grid:
            x,y = self.scene_world(event.pos())
            self.cursorWorld.emit(x,y)
            if self._press:
                self._preview = (x,y)
                self.viewport().update()
                return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if self.grid and event.button() == Qt.LeftButton:
            if self._press:
                x,y = self._press
                end = self.scene_world(event.pos())
                yaw = math.atan2(end[1]-y,end[0]-x) if math.hypot(end[0]-x,end[1]-y)>.05 else 0
                self._press = self._preview = None
                self.pointAdded.emit(x,y,yaw)
                self.viewport().update()
                return
            if self._moving >= 0:
                idx, self._moving = self._moving,-1
                self.pointMoved.emit(idx,*self.scene_world(event.pos()))
                return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event):
        if event.angleDelta().y():
            factor = 1.18 if event.angleDelta().y()>0 else 1/1.18
            zoom = self.transform().m11()*factor
            if .15 <= zoom <= 30:
                self.scale(factor,factor)
        event.accept()

    def drawForeground(self,painter,rect):
        if not self.grid:
            return
        painter.save()
        if self.grid.source=='empty':
            painter.resetTransform()
            painter.setPen(QColor('#aebfd0'))
            painter.setFont(QFont('Noto Sans CJK SC',12))
            painter.drawText(self.viewport().rect(),Qt.AlignCenter,'等待设备地图\n连接后自动读取，失败时自动重试\n也可在“设置”页手动读取')
            painter.restore()
            return
        scale = max(.01,self.transform().m11())
        unit = 1/scale
        def pixel(x,y):
            return QPointF(*self.grid.world_to_pixel(x,y))
        def line(points,color,width=2,style=Qt.SolidLine):
            pen=QPen(QColor(color),width*unit,style)
            painter.setPen(pen)
            for a,b in zip(points,points[1:]):
                painter.drawLine(pixel(*a),pixel(*b))
        if self.grid.source=='demo':
            painter.save()
            painter.setClipRect(QRectF(0,0,self.grid.width,self.grid.height))
            painter.setPen(QPen(QColor(87,118,141,30),unit))
            step=1/self.grid.resolution
            for x in range(0,self.grid.width,int(step)):
                painter.drawLine(QPointF(x,0),QPointF(x,self.grid.height))
            for y in range(0,self.grid.height,int(step)):
                painter.drawLine(QPointF(0,y),QPointF(self.grid.width,y))
            painter.restore()
        line([(p.x,p.y) for p in self.points],'#9d85ff',2,Qt.DashLine)
        line(self.path,'#00ab9e',3)
        painter.setFont(QFont('Noto Sans CJK SC',max(1,int(9*unit))))
        for i,p in enumerate(self.points):
            pt=pixel(p.x,p.y)
            rad=(13 if i==self.selected else 10)*unit
            painter.setPen(QPen(QColor('#ffffff'),2*unit))
            painter.setBrush(QColor('#7256e8' if i==self.selected else '#52668d'))
            painter.drawEllipse(pt,rad,rad)
            a=p.yaw-self.grid.origin[2]
            painter.setPen(QPen(QColor('#7256e8'),2.5*unit))
            painter.drawLine(pt+QPointF(math.cos(a)*rad,-math.sin(a)*rad),
                             pt+QPointF(math.cos(a)*25*unit,-math.sin(a)*25*unit))
            painter.setPen(QColor('white'))
            painter.drawText(QRectF(pt.x()-rad,pt.y()-rad,2*rad,2*rad),Qt.AlignCenter,str(i+1))
            painter.save()
            painter.resetTransform()
            screen=self.mapFromScene(pt)
            painter.setFont(QFont('Noto Sans CJK SC',9))
            name=painter.fontMetrics().elidedText(p.name,Qt.ElideRight,100)
            width=painter.fontMetrics().horizontalAdvance(name)+16
            tag=QRectF(screen.x()+18,screen.y()-12,width,24)
            if tag.right()>self.viewport().width()-8:
                tag.moveLeft(screen.x()-18-width)
            painter.setPen(QPen(QColor('#65758e'),1))
            painter.setBrush(QColor('#263750'))
            painter.drawRoundedRect(tag,5,5)
            painter.setPen(QColor('#f2f5fb'))
            painter.drawText(tag,Qt.AlignCenter,name)
            painter.restore()
        if self.robot:
            x,y,yaw=self.robot
            pt=pixel(x,y)
            a=yaw-self.grid.origin[2]
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(9,173,157,40))
            painter.drawEllipse(pt,.4/self.grid.resolution,.4/self.grid.resolution)
            painter.save()
            painter.translate(pt)
            painter.rotate(-math.degrees(a))
            painter.setBrush(QColor('#00a898'))
            painter.setPen(QPen(QColor('#ffffff'),2*unit))
            painter.drawPolygon(QPolygonF([QPointF(17*unit,0),QPointF(-12*unit,-10*unit),QPointF(-7*unit,0),QPointF(-12*unit,10*unit)]))
            painter.restore()
            painter.save()
            painter.setClipRect(QRectF(0,0,self.grid.width,self.grid.height))
            painter.setPen(QPen(QColor(241,147,50,190),2.2*unit))
            c,s=math.cos(yaw),math.sin(yaw)
            for lx,ly in self.scan:
                painter.drawPoint(pixel(x+c*lx-s*ly,y+s*lx+c*ly))
            painter.restore()
        if self._press and self._preview:
            line([self._press,self._preview],'#7256e8',3)
        # A screen-space scale remains legible at every map zoom level.
        painter.save()
        painter.resetTransform()
        painter.setFont(QFont('Noto Sans CJK SC',9))
        desired=90*self.grid.resolution/scale
        magnitude=10**math.floor(math.log10(max(desired,1e-9)))
        meters=min((1,2,5,10),key=lambda v:abs(v*magnitude-desired))*magnitude
        length=meters/self.grid.resolution*scale
        x,y=20,self.viewport().height()-24
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(14,27,42,230))
        painter.drawRoundedRect(QRectF(x-10,y-27,length+24,43),6,6)
        painter.setPen(QPen(QColor('#c8d8e9'),2))
        painter.drawLine(QPointF(x,y),QPointF(x+length,y))
        painter.drawLine(QPointF(x,y-4),QPointF(x,y+4))
        painter.drawLine(QPointF(x+length,y-4),QPointF(x+length,y+4))
        painter.drawText(QRectF(x,y-25,length,20),Qt.AlignCenter,'%g m'%meters)
        painter.restore()
        painter.restore()


class ScanView(QWidget):
    def __init__(self,parent=None):
        super().__init__(parent)
        self.setMinimumHeight(145)
        self.points=[]
        self.caption='等待激光数据'

    def plot_rect(self):
        # Reserve dedicated areas for the title and axis legend.
        return QRectF(8,28,max(1,self.width()-16),max(1,self.height()-53))

    def paintEvent(self,event):
        p=QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(),QColor('#111c2b'))
        plot=self.plot_rect()
        center=plot.center()
        scale=min(plot.width(),plot.height())/6
        p.save()
        p.setClipRect(plot)
        p.setPen(QPen(QColor('#2c3d50'),1))
        for radius in (1,2,3):
            p.drawEllipse(center,radius*scale,radius*scale)
        p.drawLine(QPointF(center.x(),plot.top()),QPointF(center.x(),plot.bottom()))
        p.drawLine(QPointF(plot.left(),center.y()),QPointF(plot.right(),center.y()))
        p.setPen(QPen(QColor('#f1a24a'),3))
        for x,y in self.points:
            if math.isfinite(x) and math.isfinite(y):
                p.drawPoint(QPointF(center.x()-y*scale,center.y()-x*scale))
        p.setBrush(QColor('#28cbb7'))
        p.setPen(Qt.NoPen)
        p.drawPolygon(QPolygonF([center+QPointF(0,-10),center+QPointF(-6,6),center+QPointF(6,6)]))
        p.restore()
        p.setPen(QPen(QColor('#293b50'),1))
        p.drawRect(plot)
        p.setPen(QColor('#9eafc3'))
        caption=p.fontMetrics().elidedText(self.caption,Qt.ElideRight,max(1,self.width()-20))
        p.drawText(10,17,caption)
        p.drawText(10,self.height()-7,'机体系 · 前方 ↑ · 圈距 1 m')


class TeleopPad(QWidget):
    command = pyqtSignal(float,float)
    stopped = pyqtSignal()

    def __init__(self,parent=None):
        super().__init__(parent)
        from PyQt5.QtWidgets import QVBoxLayout,QGridLayout,QPushButton,QLabel,QDoubleSpinBox,QHBoxLayout
        self.keys=set()
        self.setFocusPolicy(Qt.StrongFocus)
        self.setObjectName('teleopPad')
        layout=QVBoxLayout(self)
        note=QLabel('点击此区域后按住 W/S 前后、A/D 转向\n松键 / 失焦即停，空格停车')
        note.setWordWrap(True)
        layout.addWidget(note)
        row=QHBoxLayout()
        row.addWidget(QLabel('速度'))
        self.speed=QDoubleSpinBox()
        self.speed.setRange(.05,.5)
        self.speed.setValue(.2)
        self.speed.setSingleStep(.05)
        self.speed.setSuffix(' m/s')
        row.addWidget(self.speed)
        layout.addLayout(row)
        grid=QGridLayout()
        for text,key,r,c in [('W ↑',Qt.Key_W,0,1),('A ↶',Qt.Key_A,1,0),('停车',Qt.Key_Space,1,1),('D ↷',Qt.Key_D,1,2),('S ↓',Qt.Key_S,2,1)]:
            button=QPushButton(text)
            button.setFocusPolicy(Qt.NoFocus)
            button.pressed.connect(lambda k=key:self.press(k))
            button.released.connect(lambda k=key:self.release(k))
            grid.addWidget(button,r,c)
        layout.addLayout(grid)

    def press(self,key):
        self.setFocus(Qt.OtherFocusReason)
        if key==Qt.Key_Space:
            self.stop()
        else:
            self.keys.add(key)
            self.emit_command()

    def release(self,key):
        self.keys.discard(key)
        self.emit_command()

    def emit_command(self):
        self.command.emit(self.speed.value()*(int(Qt.Key_W in self.keys)-int(Qt.Key_S in self.keys)),
                          .5*(int(Qt.Key_A in self.keys)-int(Qt.Key_D in self.keys)))

    def stop(self):
        self.keys.clear()
        self.command.emit(0,0)
        self.stopped.emit()

    def keyPressEvent(self,e):
        if e.key() in (Qt.Key_W,Qt.Key_A,Qt.Key_S,Qt.Key_D,Qt.Key_Space):
            if not e.isAutoRepeat(): self.press(e.key())
            e.accept()
        else: super().keyPressEvent(e)

    def keyReleaseEvent(self,e):
        if e.key() in (Qt.Key_W,Qt.Key_A,Qt.Key_S,Qt.Key_D,Qt.Key_Space):
            if not e.isAutoRepeat(): self.release(e.key())
            e.accept()
        else: super().keyReleaseEvent(e)

    def focusOutEvent(self,e):
        self.stop()
        super().focusOutEvent(e)
