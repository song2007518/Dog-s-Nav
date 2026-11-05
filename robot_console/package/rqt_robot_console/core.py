"""Pure geometry, map/route validation, and vendor protocol. No device I/O."""
import json
import math
import os
import tempfile
import time
from dataclasses import dataclass, field
from pathlib import Path

import yaml
from PyQt5.QtGui import QImage, QColor


def finite(value):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError('坐标或参数不是有限数字')
    return result


def yaw_of(q):
    x, y, z, w = [finite(q.get(k, d)) for k, d in [('x', 0), ('y', 0), ('z', 0), ('w', 1)]]
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError('四元数长度为零')
    x, y, z, w = [v/norm for v in (x, y, z, w)]
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rotate_vector(q, vector):
    """Rotate a 3-D vector by normalized (x,y,z,w), including roll and pitch."""
    x,y,z,w=[finite(q[k]) for k in ('x','y','z','w')]
    norm=math.sqrt(x*x+y*y+z*z+w*w)
    if norm<1e-9: raise ValueError('四元数长度为零')
    x,y,z,w=[v/norm for v in (x,y,z,w)]
    vx,vy,vz=vector
    tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return vx+w*tx+y*tz-z*ty,vy+w*ty+z*tx-x*tz,vz+w*tz+x*ty-y*tx


def multiply_quaternion(a,b):
    x,y,z,w=[a[k] for k in ('x','y','z','w')]
    X,Y,Z,W=[b[k] for k in ('x','y','z','w')]
    return {'x':w*X+x*W+y*Z-z*Y,'y':w*Y-x*Z+y*W+z*X,
            'z':w*Z+x*Y-y*X+z*W,'w':w*W-x*X-y*Y-z*Z}


@dataclass
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0

    def pose(self):
        return {'position': {'x': self.x, 'y': self.y, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(self.yaw/2), 'w': math.cos(self.yaw/2)}}

    def record(self):
        return {'name': self.name, 'x': self.x, 'y': self.y, 'yaw': self.yaw}


@dataclass
class GridMap:
    map_id: str
    width: int
    height: int
    resolution: float
    origin: tuple
    cells: list  # ROS occupancy, bottom row first; -1 unknown, 0..100 occupancy
    frame: str = 'map'
    source: str = 'local'

    def __post_init__(self):
        self.resolution = finite(self.resolution)
        self.origin = tuple(finite(x) for x in self.origin)
        if len(self.origin) != 3 or self.width <= 0 or self.height <= 0 or self.resolution <= 0:
            raise ValueError('地图尺寸、原点或分辨率无效')
        if self.width*self.height > 16_000_000 or len(self.cells) != self.width*self.height:
            raise ValueError('地图数据长度与尺寸不一致或地图过大')

    def world_to_cell_float(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (c*dx+s*dy)/self.resolution, (-s*dx+c*dy)/self.resolution

    def cell_to_world(self, x, y):
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (self.origin[0]+self.resolution*(c*x-s*y),
                self.origin[1]+self.resolution*(s*x+c*y))

    def world_to_pixel(self, x, y):
        gx, gy = self.world_to_cell_float(x, y)
        return gx, self.height-gy

    def pixel_to_world(self, x, y):
        return self.cell_to_world(x, self.height-y)

    def occupancy(self, x, y):
        gx, gy = self.world_to_cell_float(x, y)
        ix, iy = math.floor(gx), math.floor(gy)
        if not (0 <= ix < self.width and 0 <= iy < self.height):
            return None
        return self.cells[iy*self.width+ix]

    def valid_point(self, x, y):
        val = self.occupancy(x, y)
        return val is not None and 0 <= val < 50

    def image(self):
        colors = [QColor('#536174').rgba()] + [QColor(232-round(v*1.93), 238-round(v*1.96), 243-round(v*1.98)).rgba() for v in range(101)]
        raw = bytes(v+1 for row in range(self.height-1, -1, -1)
                    for v in self.cells[row*self.width:(row+1)*self.width])
        img = QImage(raw, self.width, self.height, self.width, QImage.Format_Indexed8).copy()
        img.setColorTable(colors)
        return img

    @classmethod
    def from_yaml(cls, path):
        path = Path(path)
        info = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(info, dict):
            raise ValueError('地图 YAML 必须是对象')
        img_path = path.parent / str(info['image'])
        img = QImage(str(img_path))
        if img.isNull():
            raise ValueError('无法读取地图图片：'+str(img_path))
        if img.width()*img.height() > 16_000_000:
            raise ValueError('地图超过 1600 万格限制')
        free, occupied = finite(info.get('free_thresh', .196)), finite(info.get('occupied_thresh', .65))
        if not 0 <= free < occupied <= 1:
            raise ValueError('地图占用阈值无效')
        negate = int(info.get('negate', 0))
        cells = []
        for y in range(img.height()-1, -1, -1):
            for x in range(img.width()):
                color = img.pixelColor(x, y)
                value = (color.red()+color.green()+color.blue())/765.0
                occ = value if negate else 1-value
                cells.append(-1 if color.alpha() < 255 else (100 if occ > occupied else (0 if occ < free else -1)))
        return cls(path.stem, img.width(), img.height(), info['resolution'], tuple(info['origin']), cells)

    @classmethod
    def from_payload(cls, map_id, payload, encoding='occupancy'):
        info = payload['info']
        values = payload['data']
        if not isinstance(values, list):
            raise ValueError('地图 data 应为数组')
        # The vendor manual calls this a byte array, but local code expects ROS occupancy.
        # Never silently clamp 255 to 100: operator must choose the actual encoding.
        if encoding == 'occupancy':
            if any(type(v) is not int or not -1 <= v <= 100 for v in values):
                raise ValueError('地图不是 -1..100 占用栅格；请核实并切换为 PGM 灰度编码')
            cells = values[:]
        else:
            if any(type(v) is not int or not 0 <= v <= 255 for v in values):
                raise ValueError('PGM 灰度值须在 0..255')
            cells = [100 if v < 90 else (0 if v > 205 else -1) for v in values]
        origin = info.get('origin', {})
        pos = origin.get('position', {})
        return cls(str(map_id), int(info['width']), int(info['height']), info['resolution'],
                   (pos.get('x', 0), pos.get('y', 0), yaw_of(origin.get('orientation', {}))), cells,
                   payload.get('header', {}).get('frame_id', 'map'), 'robot')

    def export(self, yaml_path):
        path = Path(yaml_path)
        img_path = path.with_suffix('.pgm')
        raw = bytes(205 if v < 0 else round(254*(1-v/100))
                    for row in range(self.height-1, -1, -1)
                    for v in self.cells[row*self.width:(row+1)*self.width])
        img_path.write_bytes(('P5\n%d %d\n255\n' % (self.width, self.height)).encode()+raw)
        path.write_text(yaml.safe_dump({'image': img_path.name, 'resolution': self.resolution,
                        'origin': list(self.origin), 'negate': 0, 'occupied_thresh': .65, 'free_thresh': .196},
                        sort_keys=False), encoding='utf-8')


def demo_map():
    w, h = 200, 150
    cells = [0]*(w*h)
    for y in range(h):
        for x in range(w):
            wall = x < 3 or x >= w-3 or y < 3 or y >= h-3
            blocks = (55 <= x <= 78 and 45 <= y <= 108) or (122 <= x <= 150 and 25 <= y <= 76)
            if wall or blocks:
                cells[y*w+x] = 100
    return GridMap('demo_office', w, h, .05, (-5, -3.75, 0), cells, source='demo')


def save_route(path, map_id, route_id, points):
    validate_points(points)
    data = {'schema_version': 1, 'map_id': map_id, 'route_id': route_id,
            'coordinate_frame': 'map', 'yaw_unit': 'radian', 'points': [p.record() for p in points]}
    atomic_json(path, data)


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(path.parent), prefix='.'+path.name)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False, allow_nan=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def validate_points(points, grid=None):
    if not points or len(points) > 1000:
        raise ValueError('巡航点数量应为 1～1000')
    names = set()
    for p in points:
        if not p.name.strip() or p.name in names:
            raise ValueError('巡航点名称不能为空或重复')
        names.add(p.name)
        for v in (p.x, p.y, p.yaw):
            finite(v)
        if grid and not grid.valid_point(p.x, p.y):
            raise ValueError('%s 位于障碍、未知区域或地图外' % p.name)


def load_route(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if data.get('schema_version') != 1 or data.get('yaw_unit') != 'radian' or data.get('coordinate_frame') != 'map':
        raise ValueError('不支持的路线版本、坐标系或角度单位')
    points = [Waypoint(str(p['name']), finite(p['x']), finite(p['y']), finite(p['yaw'])) for p in data['points']]
    validate_points(points)
    if not isinstance(data.get('map_id'), str) or not isinstance(data.get('route_id'), str):
        raise ValueError('地图和路线 ID 必须是字符串')
    return data['map_id'], data['route_id'], points


def request(func, value, seq):
    return {'head': {'type': 'app_req', 'time_stamp': int(time.time()*1000),
                     'source': 'app', 'frame_count': seq}, 'data': {'req_func': {func: value}}}


ALIASES = {'loc_load_map': {'loc_load_map', 'load_localization_map'}}


def response_for(message, func, seq):
    if message.get('head', {}).get('type') != 'app_resp':
        return None
    count = message.get('head', {}).get('frame_count')
    if count is not None and str(count) != str(seq):
        return None
    payload = message.get('data', {}).get('req_result', {})
    if not isinstance(payload, dict):
        return None
    for key in ('AppResponse', 'AppMapListResponse', 'AppReponseObjectData', 'AppResponseObjectData'):
        if isinstance(payload.get(key), dict):
            payload = payload[key]
            break
    if payload.get('req_func') not in ALIASES.get(func, {func}):
        return None
    return payload


def path_payload(map_id, route_id, points):
    validate_points(points)
    return [map_id, route_id, [[p.name, p.pose()] for p in points]]


def demo_plan(grid, start, goal, radius=.25):
    """Collision-checked A* and smoothing for the labelled offline simulator."""
    from .planning import plan
    return plan(grid, start, goal, radius)
