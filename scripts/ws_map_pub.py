#!/usr/bin/env python3
# ws_map_pub.py
# 从伴算机 WebSocket 拉取当前定位地图(OccupancyGrid)并发布 /map
#   - 关键: 每次"连接->取图->关闭"短连接, 不长期占用 App 连接槽
#     (算法管理器只允许一个 App 连接; 若长期占用会顶掉 loc_load_map/建图等)
#   - 默认发布最新地图; 用 --map-id 可锁定某张
#   - TRANSIENT_LOCAL 发布, 迟到的订阅者也能拿到
# 用法:
#   python3 ws_map_pub.py [--map-id <map_id>]
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import websocket

WS_URL = "ws://192.168.234.234:10010"
FETCH_PERIOD = 10.0


def make_req(func_name, value):
    return {
        "head": {"type": "app_req", "time_stamp": int(time.time() * 1000),
                 "source": "app", "frame_count": 1},
        "data": {"req_func": {func_name: value}},
    }


def pick_newest_id(map_data):
    def ctime(mid):
        arr = map_data.get(mid) or []
        return (arr[0] or {}).get("create_time_ms", 0) if arr else 0
    return max(map_data, key=ctime)


def fetch_occupancy(node, want_id=None):
    """Short-lived connection: connect -> request -> close. Returns (map_id, payload, err)."""
    ws = None
    try:
        ws = websocket.create_connection(WS_URL, timeout=6)
        ws.send(json.dumps(make_req("get_all_pgm_map", None)))
        deadline = time.time() + 8
        obj = None
        while time.time() < deadline:
            m = json.loads(ws.recv())
            if not isinstance(m, dict) or m.get("head", {}).get("type") != "app_resp":
                continue
            rr = m.get("data", {}).get("req_result", {})
            for k in ("AppResponse", "AppMapListResponse"):
                if k in rr:
                    obj = rr[k]
                    break
            if obj is not None:
                break
        if obj is None:
            return None, None, "no app_resp from ws"
        maps = obj.get("data") or {}
        if not isinstance(maps, dict) or not maps:
            return None, None, "no map on board"
        map_id = want_id if (want_id and want_id in maps) else pick_newest_id(maps)
        if want_id and want_id not in maps:
            node.get_logger().warn("map id %s not on board, use newest %s" % (want_id, map_id))
        arr = maps.get(map_id) or []
        if not arr:
            return None, None, "empty map entry %s" % map_id
        e = arr[0]
        info = e.get("info") or {}
        data = e.get("data") or []
        header = e.get("header") or {}
        w, h = int(info.get("width", 0)), int(info.get("height", 0))
        if w * h <= 0 or len(data) != w * h:
            return None, None, "bad map dims %dx%d data=%d" % (w, h, len(data))
        return map_id, (info, header, data), None
    except Exception as ex:
        return None, None, str(ex)
    finally:
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass


def build_msg(info, header, data):
    occ = OccupancyGrid()
    occ.header.frame_id = header.get("frame_id", "map")
    occ.info.map_load_time.sec = int(time.time())
    occ.info.resolution = float(info.get("resolution", 0.05))
    occ.info.width = int(info.get("width", 0))
    occ.info.height = int(info.get("height", 0))

    pos = info.get("origin", {}).get("position", {})
    ori = info.get("origin", {}).get("orientation", {})
    occ.info.origin.position.x = float(pos.get("x", 0.0))
    occ.info.origin.position.y = float(pos.get("y", 0.0))
    occ.info.origin.position.z = float(pos.get("z", 0.0))
    occ.info.origin.orientation.x = float(ori.get("x", 0.0))
    occ.info.origin.orientation.y = float(ori.get("y", 0.0))
    occ.info.origin.orientation.z = float(ori.get("z", 0.0))
    occ.info.origin.orientation.w = float(ori.get("w", 1.0))

    occ.data = [max(-1, min(100, int(v))) for v in data]
    return occ


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map-id", default=None)
    args = ap.parse_args()

    rclpy.init()
    node = Node("ws_map_pub")
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    pub = node.create_publisher(OccupancyGrid, "/map", qos)

    current_id = None
    current_key = None
    failures = 0

    while rclpy.ok():
        map_id, payload, err = fetch_occupancy(node, want_id=args.map_id)
        if payload is not None:
            failures = 0
            info, header, data = payload
            # 只在图变化时重建并发布(TRANSIENT_LOCAL 已保持旧图给订阅者)
            key = (map_id, len(data), tuple(data[::97]))
            if key != current_key:
                msg = build_msg(info, header, data)
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                current_id = map_id
                current_key = key
                node.get_logger().info(
                    "map %s  %dx%d res=%.2f origin=(%.2f,%.2f)" %
                    (map_id, msg.info.width, msg.info.height,
                     msg.info.resolution,
                     msg.info.origin.position.x, msg.info.origin.position.y))
        else:
            failures += 1
            if failures % 3 == 1:
                node.get_logger().warn("fetch map failed: %s" % err)
        # 短周期休眠, 期间 App 连接槽完全空闲(供 loc_load_map/建图等使用)
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(FETCH_PERIOD)

    rclpy.shutdown()


if __name__ == "__main__":
    main()