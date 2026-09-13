import json
import sys
import time
import websocket

WS_URL = "ws://192.168.234.234:10010"


def make_req(func_name, value):
    return {
        "head": {"type": "app_req", "time_stamp": int(time.time() * 1000),
                 "source": "app", "frame_count": 1},
        "data": {"req_func": {func_name: value}},
    }


def send(ws, func, val, wait=5.0, label=None):
    ws.send(json.dumps(make_req(func, val)))
    print("== %s ==" % (label or func))
    deadline = time.time() + wait
    while time.time() < deadline:
        try:
            msg = ws.recv()
        except Exception:
            return
        m = json.loads(msg)
        if m.get("head", {}).get("type") == "app_resp":
            rr = m.get("data", {}).get("req_result", {})
            for k in ("AppResponse", "AppMapListResponse"):
                if k in rr:
                    obj = rr[k]
                    print("  %s: %s | data=%s" % (obj.get("req_func"), obj.get("status"),
                                                  str(obj.get("data"))[:120]))
                    return
            print("  raw:", msg[:250])
            return


def recv_resp(ws, wait=8.0):
    """Fetch the first app_resp payload (AppResponse / AppMapListResponse)."""
    deadline = time.time() + wait
    while time.time() < deadline:
        try:
            msg = json.loads(ws.recv())
        except Exception:
            return None
        m = msg if isinstance(msg, dict) else None
        if m is None or m.get("head", {}).get("type") != "app_resp":
            continue
        rr = m.get("data", {}).get("req_result", {})
        for k in ("AppResponse", "AppMapListResponse"):
            if k in rr:
                return rr[k]
    return None


def main():
    raw = "--raw" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--raw"]
    if not args:
        print("usage: ws_cmd.py [--raw] <func> [val]")
        return
    func = args[0]
    val = args[1] if len(args) > 1 else None
    try:
        val = json.loads(val) if isinstance(val, str) and val.startswith(("{", "[")) else val
    except Exception:
        pass
    ws = websocket.create_connection(WS_URL, timeout=10)
    if raw:
        ws.send(json.dumps(make_req(func, val)))
        obj = recv_resp(ws)
        print(json.dumps(obj, ensure_ascii=False) if obj is not None else "null")
    else:
        print("connected")
        send(ws, func, val)
    time.sleep(1)
    ws.close()


if __name__ == "__main__":
    main()
