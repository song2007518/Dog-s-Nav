#!/usr/bin/env python3
# 决定性诊断: 用原始 /front_lidar 点云(过滤掉地面后投影2D) 匹配地图
# 判断: 地图过时(匹配差) vs 定位失效(匹配好但 loc_load_map 不收敛)
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import websocket
import json

WS_URL = "ws://192.168.234.234:10010"
EX, EY, EZ = 0.20985, 0.0, 0.16952


def make_req(f, v):
    return {"head": {"type": "app_req", "time_stamp": int(time.time() * 1000),
                     "source": "app", "frame_count": 1},
            "data": {"req_func": {f: v}}}


def fetch_map():
    ws = websocket.create_connection(WS_URL, timeout=6)
    ws.send(json.dumps(make_req("get_all_pgm_map", None)))
    deadline = time.time() + 8
    obj = None
    while time.time() < deadline:
        m = json.loads(ws.recv())
        if isinstance(m, dict) and m.get("head", {}).get("type") == "app_resp":
            rr = m["data"].get("req_result", {})
            for k in ("AppResponse", "AppMapListResponse"):
                if k in rr:
                    obj = rr[k]; break
            if obj: break
    ws.close()
    maps = (obj or {}).get("data") or {}
    mid = max(maps, key=lambda m: (maps[m][0] or {}).get("create_time_ms", 0) if maps[m] else 0)
    return mid, maps[mid][0]["info"], maps[mid][0]["data"]


def get_cloud_2d():
    rclpy.init()
    n = Node("cloud_match")
    got = []
    n.create_subscription(PointCloud2, "/front_lidar", lambda m: got.append(m), 5)
    t0 = time.time()
    while time.time() - t0 < 6 and not got:
        rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.shutdown()
    if not got:
        return None
    m = got[0]
    off = {f.name: f.offset for f in m.fields if f.name in ("x", "y", "z") and f.datatype == 7}
    step = m.point_step or 16
    npc = len(m.data) // step
    pts = []
    for i in range(npc):
        b = m.data[i * step:(i + 1) * step]
        x = struct.unpack_from("<f", b, off["x"])[0]
        y = struct.unpack_from("<f", b, off["y"])[0]
        z = struct.unpack_from("<f", b, off["z"])[0]
        bx, by, bz = x + EX, y + EY, z + EZ
        if math.hypot(bx, by) < 0.4:
            continue
        h = bz - EZ  # 高度相对雷达平面
        if not (0.1 <= h <= 1.0):
            continue  # 只留墙/障碍, 排除地面
        if bx < 0.1:
            continue
        pts.append((bx, by))
    return pts


def match_score(pts, info, data, x, y, yaw):
    w, h = info["width"], info["height"]
    res = info["resolution"]
    ox = info["origin"]["position"]["x"]
    oy = info["origin"]["position"]["y"]
    c, s = math.cos(yaw), math.sin(yaw)
    hit = 0
    for (lx, ly) in pts:
        wx = x + c * lx - s * ly
        wy = y + s * lx + c * ly
        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)
        if not (0 <= cx < w and 0 <= cy < h):
            continue
        if data[cy * w + cx] >= 50:
            hit += 1
    return hit / len(pts)


def main():
    mid, info, data = fetch_map()
    print("地图 %s  %dx%d" % (mid, info["width"], info["height"]))
    pts = get_cloud_2d()
    if not pts:
        print("无点云"); return
    print("原始点云过滤后点数: %d" % len(pts))
    print("\n=== 匹配度(点云命中地图占用格) ===")
    best = (0, None)
    for cx in range(-4, 5):
        for cy in range(-4, 5):
            px, py = 8.8 + cx * 0.3, 3.7 + cy * 0.3
            for dyaw in range(0, 360, 30):
                sc = match_score(pts, info, data, px, py, math.radians(dyaw))
                if sc > best[0]:
                    best = (sc, (px, py, dyaw))
    sc, (px, py, yaw) = best
    print("最佳: pos=(%.2f,%.2f) yaw=%ddeg  匹配=%.3f" % (px, py, yaw, sc))
    if sc >= 0.7:
        print("判定: 环境匹配良好(>=0.7) -> 定位算法失效, 不用重新建图")
    elif sc >= 0.55:
        print("判定: 中等匹配(0.55-0.7) -> 边界情况, 可试手动定位或重试")
    else:
        print("判定: 匹配差(<0.55) -> 环境/地图对不上(地图过时或狗位置不对), 需要重新建图")


if __name__ == "__main__":
    main()