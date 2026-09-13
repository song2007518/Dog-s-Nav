#!/usr/bin/env python3
# cam_view.py
# 前视相机 RTSP 拉流显示 (ZSL-1(1W): rtsp://192.168.234.1:8554/test)
# 深度相机走 NX 的 ROS2 话题 (/rs_pointcloud, /rs_ir_left/compressed), 本脚本不管
# 用法:
#   python3 cam_view.py [--url rtsp://192.168.234.1:8554/test] [--scale 0.5]
#   ESC / 窗口关闭 退出; 断流自动重连
import argparse
import os
import sys
import time

import cv2

DEFAULT_URL = "rtsp://192.168.234.1:8554/test"
WIN_NAME = "front_cam"
MARKER = "/tmp/custom_nav_launch/cam_view.user_closed"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--scale", type=float, default=0.5,
                    help="显示缩放(1080p 默认 0.5 -> 960x540)")
    args = ap.parse_args()

    # 手动/监控拉起 = 用户想开相机, 清掉"用户已关闭"标记
    if os.path.exists(MARKER):
        try:
            os.remove(MARKER)
        except OSError:
            pass

    print("cam_view: 拉流 %s (ESC / 点窗口 X 关闭)" % args.url)
    cap = None
    consecutive_fail = 0
    user_closed = False
    was_visible = False       # 窗口是否曾经真正显示出来
    while True:
        if cap is None:
            cap = cv2.VideoCapture(args.url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                cap = None
                print("[%s] 连接失败, 3s 后重试 ..." % time.strftime("%H:%M:%S"))
                time.sleep(3)
                continue
            # 只读一帧的流式(非录制)参数
            cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 4000)
            cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 4000)
            print("已连接")

        ok, frame = cap.read()
        if not ok:
            consecutive_fail += 1
            if consecutive_fail >= 3:
                print("读取失败, 断流重连 ...")
                cap.release()
                cap = None
                consecutive_fail = 0
                time.sleep(2)
            continue
        consecutive_fail = 0

        h, w = frame.shape[:2]
        dw, dh = int(w * args.scale), int(h * args.scale)
        show = cv2.resize(frame, (dw, dh)) if args.scale != 1.0 else frame
        cv2.imshow(WIN_NAME, show)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            user_closed = True
            break
        # 窗口被点 X 关闭: 只有"曾经显示过"再变不可见才认为是用户关闭,
        # 避免窗口尚未渲染完成就误判退出(一闪就关)
        try:
            visible = cv2.getWindowProperty(WIN_NAME, cv2.WND_PROP_VISIBLE)
            if visible >= 1:
                was_visible = True
            elif was_visible:
                user_closed = True
                break
        except cv2.error:
            if was_visible:
                user_closed = True
                break

    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()

    # 用户主动关闭 -> 写标记, 让 start_all 监控不再自动重启
    if user_closed:
        try:
            os.makedirs(os.path.dirname(MARKER), exist_ok=True)
            with open(MARKER, "w"):
                pass
        except OSError:
            pass
        print("相机窗口已关闭(用户), 已标记不再自动重启")
    return 0


if __name__ == "__main__":
    main()