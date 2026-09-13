#!/usr/bin/env python3
# 诊断: 当前激光扫描 vs 保存地图, 在候选位姿下的匹配度
# 用法: python3 scan_map_match.py [--pose x y yaw]
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import websocket

WS_URL = "ws://192.168.234.234:10010"


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
    e = maps[mid][0]
    info = e["info"]
    return mid, info, e["data"]


def score_scan_at(map_, info, data, scan, x, y, yaw):
    w, h = info["width"], info["height"]
    res = info["resolution"]
    ox = info["origin"]["position"]["x"]
    oy = info["origin"]["position"]["y"]
    c, s = math.cos(yaw), math.sin(yaw)
    hit = total = 0
    amin = scan.angle_min
    ainc = scan.angle_increment
    for i, r in enumerate(scan.ranges):
        if not (0.4 <= r <= 12.0):
            continue
        a = amin + i * ainc
        lx, ly = r * math.cos(a), r * math.sin(a)  # base_link frame
        wx = x + c * lx - s * ly
        wy = y + s * lx + c * ly
        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)
        if not (0 <= cx < w and 0 <= cy < h):
            continue
        total += 1
        v = data[cy * w + cx]
        if v >= 50:
            hit += 1
        elif v >= 0 and v < 50:  # free cell: scan SHOULD pass -> mismatch if we hit an object here
            pass
    return (hit / total) if total else 0.0, hit, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pose", nargs=3, type=float, default=[8.3, 2.4, 0.0],
                    help="x y yaw(deg)")
    args = ap.parse_args()
    px, py, pyaw_deg = args.pose
    pyaw = math.radians(pyaw_deg)

    rclpy.init()
    n = Node("scan_map_match")
    scan = []
    n.create_subscription(LaserScan, "/laser_scan", lambda m: scan.append(m), 5)
    t0 = time.time()
    while time.time() - t0 < 6 and not scan:
        rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.shutdown()
    if not scan:
        print("无 /laser_scan")
        return
    s = scan[0]
    print("scan frame=%s rays=%d" % (s.header.frame_id, len(s.ranges)))

    mid, info, data = fetch_map()
    w, h = info["width"], info["height"]
    print("map %s %dx%d res=%.2f origin=(%.2f,%.2f)" %
          (mid, w, h, info["resolution"], info["origin"]["position"]["x"], info["origin"]["position"]["y"]))

    # 测试一组位姿: 给定位姿 + 绕 360 步进
    print("\n=== 匹配度 (命中占用率) ===")
    best = (0, None)
    for dyaw in range(0, 360, 30):
        yy = pyaw + math.radians(dyaw)
        sc, hit, tot = score_scan_at(info, info, data, s, px, py, yy)
        if tot > 30 and sc > best[0]:
            best = (sc, (px, py, yy))
        print("  pos(%.1f,%.1f) yaw=%3d deg: 命中=%.3f (%d/%d)" % (px, py, dyaw, sc, hit, tot))
    if best[1]:
        print("\n最优: yaw=%.1fdeg score=%.3f" % (math.degrees(best[1][2]), best[0]))


if __name__ == "__main__":
    main()