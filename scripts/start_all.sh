#!/bin/bash
# ============================================================
# custom_nav 一键启动 + 自愈监控 (PC 端)
# 用法:
#   ./start_all.sh              前台运行: 拉起 + 常驻自愈, Ctrl+C 安全趴下
#   ./start_all.sh --boot       后台守护 (登录自启 / 无人值守):
#                                 日志见 /tmp/custom_nav_launch/boot.log
#   ./start_all.sh --stop       停止后台守护 (自动受控趴下)
# 自动完成 (重启后无需手动操作):
#   zenoh -> 板子驱动就绪 -> 自动站立 -> 定位恢复(自动探测最新地图ID)
#   -> /map (ws_map_pub 直拉板上真图, 无需 scp) -> custom_nav -> rviz
# 自愈监控 (脚本运行期间):
#   - 狗重启 / 驱动掉线 / 定位卡死(mc_odom) -> 自动重新站立 + 定位恢复
#   - custom_nav / rviz / ws_map_pub 意外退出 -> 自动拉起
# 之后只需: rviz 发 2D Goal -> 狗自动使能并自主导航过去
# 急停(任意时刻, 前台): python3 src/custom_nav/scripts/e_stop.py [按 q]
# ============================================================

set +e

# ---------- 0. 配置 ----------
ROOT=/home/song/robot_ws/robot_sdk_ws
SCRIPTS=$ROOT/scripts
ROS_SETUP=/opt/ros/humble/setup.bash
WS_SETUP=$ROOT/install/setup.bash
ZENOH_CONFIG=~/.config/zenoh/pc_router.json5
RViz_CONFIG=~/.config/robot_nav/rviz_readonly.rviz
MAP_ID_FALLBACK="2026_08_08_07_28_38"

BOARD_IP=192.168.234.1
BOARD_USER=firefly
BOARD_PASS=firefly
COMP_IP=192.168.234.234

export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

LOG_DIR=/tmp/custom_nav_launch
PID_FILE=$LOG_DIR/boot.pid
mkdir -p "$LOG_DIR"

# 全局: 当前使用的定位地图ID (自动探测, 空=未知)
MAP_ID=""

# ---------- 0.5 模式解析 (--boot / --stop) ----------
BOOT=0
case "$1" in
  --boot) BOOT=1 ;;
  --stop)
    if [ -f "$PID_FILE" ]; then
      BPID=$(cat "$PID_FILE")
      if [ -n "$BPID" ] && kill -0 "$BPID" 2>/dev/null; then
        echo "发送停止信号给守护 (pid $BPID) -> 狗将受控趴下 ..."
        kill -TERM "$BPID"
        for _ in $(seq 1 20); do
          kill -0 "$BPID" 2>/dev/null || break
          sleep 1
        done
        kill -0 "$BPID" 2>/dev/null && echo "  (守护未退出, 可再执行一次)" || rm -f "$PID_FILE"
      else
        echo "守护不在运行, 清理残留 pid 文件"
        rm -f "$PID_FILE"
      fi
    else
      echo "没有运行中的后台守护"
    fi
    exit 0 ;;
esac

# --boot 且尚未分离 -> 派生后台子进程后立即返回
if [ "$BOOT" = "1" ] && [ -z "$RBN_DETACHED" ]; then
  if [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE" 2>/dev/null)" 2>/dev/null; then
    echo "已有一个后台守护在运行 (pid $(cat "$PID_FILE"))"
    exit 1
  fi
  echo "以后台守护方式启动 ... (日志: $LOG_DIR/boot.log)"
  RBN_DETACHED=1 setsid nohup bash "$0" --boot \
    > "$LOG_DIR/boot.log" 2>&1 < /dev/null &
  exit 0
fi

source "$ROS_SETUP"
source "$WS_SETUP"

# ---------- 工具函数 ----------
now() { date +%T; }
log() { echo "[$(now)] $*"; }

zs1w_online() { timeout 8 ros2 service list 2>/dev/null | grep -q "zs1w/stand_up"; }
board_ping() { ping -c1 -W1 -q 192.168.234.1 >/dev/null 2>&1; }

odom_is_localized() {
  timeout 6 ros2 topic echo /tf_manager/odom_type --once 2>/dev/null | grep -q "localization_odom"
}

# 探测伴算机上最新地图ID (定位恢复与 /map 都用它). 失败时保留原 MAP_ID
detect_map_id() {
  local raw id
  raw=$(timeout 20 python3 "$SCRIPTS/ws_cmd.py" --raw get_all_pgm_map 2>/dev/null)
  [ -z "$raw" ] && return 1
  id=$(printf '%s' "$raw" | timeout 10 python3 -c '
import sys, json
try:
    o = json.load(sys.stdin)
    d = (o or {}).get("data")
    if not isinstance(d, dict) or not d:
        sys.exit(1)
    def ctime(mid):
        a = d.get(mid) or []
        return (a[0] or {}).get("create_time_ms", 0) if a else 0
    print(max(d, key=ctime))
except Exception:
    sys.exit(1)
' 2>/dev/null)
  if [ -n "$id" ]; then
    MAP_ID="$id"
    return 0
  fi
  return 1
}

# 拉驱动 (狗在线但驱动未运行时 SSH 到板子启动)
ensure_driver() {
  if zs1w_online; then
    log "板子 zs1w 服务已在线, 跳过驱动启动"
    return 0
  fi
  if ! board_ping; then
    log "板子 ($BOARD_IP) ping 不通, 驱动无法启动 (狗未开机?)"
    return 1
  fi
  log "zs1w 服务不可用, SSH 到板子启动驱动 ..."
  sshpass -p "$BOARD_PASS" ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 \
    "$BOARD_USER@$BOARD_IP" '
    if [ ! -f /tmp/start_zs1w.sh ]; then
      cat > /tmp/start_zs1w.sh << "EOF"
#!/bin/bash
export ROS_DOMAIN_ID=24
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
source /opt/ros/humble/setup.bash
source /home/firefly/genisom_l1_sdk_old/demo/zsl-1w/ros2/ws/install/setup.bash
export LD_LIBRARY_PATH=/home/firefly/genisom_l1_sdk_old/demo/zsl-1w/ros2/ws/install/zs1w_driver/lib/zs1w_driver:$LD_LIBRARY_PATH
exec ros2 run zs1w_driver zs1w_driver_node --ros-args \
  -p local_ip:=192.168.234.1 -p local_port:=43988 -p dog_ip:=192.168.234.1
EOF
      chmod +x /tmp/start_zs1w.sh
    fi
    setsid nohup /tmp/start_zs1w.sh > /tmp/zs1w_driver.log 2>&1 < /dev/null &
    echo "driver started"
  '
  for _ in $(seq 1 15); do
    zs1w_online && break
    sleep 1
  done
  if zs1w_online; then
    log "zs1w 驱动已就绪"
    return 0
  else
    log "!! zs1w 驱动仍未就绪 (板子查 /tmp/zs1w_driver.log)"
    return 1
  fi
}

auto_stand() {
  if ! zs1w_online; then
    log "驱动未在线, 跳过站立"
    return 1
  fi
  log "自动站立 (stand_up) ..."
  timeout 10 ros2 service call /zs1w/stand_up std_srvs/srv/Trigger 2>&1 | grep -qi "success" \
    && log "  ✓ 站立成功" || log "  (stand_up 调用失败或已在站立状态)"
}

# 定位恢复: reset_loc -> MapState(data=1) -> loc_load_map(最新地图)
# 探测不到地图ID时不执行(避免 reset_loc 后无图可载), 返回 1
# 注意: MapState.srv 请求只有 data 一个字段 (FAIL->STANDBY)
do_localization_recover() {
  if ! detect_map_id; then
    if [ -z "$MAP_ID" ]; then
      log "  (无法探测板上地图ID, 暂不执行恢复, 稍后自动重试)"
      return 1
    fi
    log "  (地图探测失败, 沿用已缓存地图: $MAP_ID)"
  else
    log "使用地图: $MAP_ID"
  fi

  log "执行定位恢复序列 ..."
  # ws_map_pub 用短连接取图, 恢复命令可能偶发撞车 -> 各自重试几次
  for _ in $(seq 1 4); do
    python3 "$SCRIPTS/ws_cmd.py" reset_loc 2>/dev/null | grep -q "ok" && break
    sleep 1
  done
  sleep 1
  timeout 10 ros2 service call /robot_slam/localization_state_service \
    robots_dog_msgs/srv/MapState \
    "{data: 1}" \
    > /dev/null 2>&1
  sleep 1
  for _ in $(seq 1 6); do
    python3 "$SCRIPTS/ws_cmd.py" loc_load_map "$MAP_ID" 2>/dev/null | grep -q "ok" && break
    sleep 1
  done
  sleep 3
}

# 等待 /map 实际发布: 保证 custom_nav 起来时能拿到图
wait_map_ready() {
  for _ in $(seq 1 10); do
    timeout 5 ros2 topic echo /map --once > /dev/null 2>&1 && return 0
    sleep 1
  done
  return 1
}

# 启动 ws_map_pub (直拉板上真图). 传参 MAP_ID 锁定地图.
ensure_ws_map_pub() {
  local args=""
  [ -n "$MAP_ID" ] && args="--map-id $MAP_ID"
  # 已在运行且地图一致 -> 不动
  if pgrep -f "ws_map_pub.py" > /dev/null 2>&1; then
    if [ -n "$MAP_ID" ]; then
      pgrep -f "ws_map_pub.py --map-id $MAP_ID" > /dev/null 2>&1 && return 0
      log "地图已更新, 重启 ws_map_pub (锁定 $MAP_ID)"
      pkill -f "ws_map_pub.py" 2>/dev/null
      sleep 1
    else
      return 0
    fi
  fi
  setsid nohup python3 "$SCRIPTS/ws_map_pub.py" $args \
    > "$LOG_DIR/ws_map_pub.log" 2>&1 < /dev/null &
  sleep 2
  if pgrep -f "ws_map_pub.py" > /dev/null 2>&1; then
    log "ws_map_pub 已启动 (发布 /map) $args"
  else
    log "!! ws_map_pub 启动失败 (见 $LOG_DIR/ws_map_pub.log)"
  fi
}

ensure_custom_nav() {
  pgrep -f "custom_nav_node" > /dev/null 2>&1 && return 0
  log "启动 custom_nav (auto_enable_on_goal) ..."
  # 加载 config/custom_nav.yaml 全部参数(障碍膨胀/重规划频率/急停距离等),
  # 再以 CLI 覆盖运行时参数。以后调参只改 yaml, 无需重新编译。
  setsid nohup ros2 run custom_nav custom_nav_node --ros-args \
    --params-file "$ROOT/src/custom_nav/config/custom_nav.yaml" \
    -p auto_standup:=false -p motion_enabled:=false \
    -p auto_enable_on_goal:=true \
    > "$LOG_DIR/custom_nav.log" 2>&1 < /dev/null &
  sleep 2
}

ensure_rviz() {
  pgrep -f "rviz2" > /dev/null 2>&1 && return 0
  log "启动 rviz ..."
  setsid nohup rviz2 -d "$RViz_CONFIG" > "$LOG_DIR/rviz.log" 2>&1 < /dev/null &
  sleep 2
}

# 前置相机桥: 压缩流转发 + camera_info (rviz 显示相机用)
# 前视相机 RTSP 拉流显示窗口 (OpenCV, 不依赖 rviz 的 image 显示)
# 用户关窗(ESC/X)后写 cam_view.user_closed 标记 -> 不再自动重启; 下次 start_all 重新打开
ensure_cam_view() {
  [ -f "$LOG_DIR/cam_view.user_closed" ] && return 0
  pgrep -f "cam_view.py" > /dev/null 2>&1 && return 0
  log "启动 cam_view (RTSP 相机窗口) ..."
  setsid nohup python3 "$SCRIPTS/cam_view.py" \
    > "$LOG_DIR/cam_view.log" 2>&1 < /dev/null &
  sleep 3
  pgrep -f "cam_view.py" > /dev/null 2>&1 \
    && log "  ✓ cam_view 已启动" \
    || log "  !! cam_view 启动失败 (见 $LOG_DIR/cam_view.log)"
}

# ---------- 1. 清理残留进程 (确保干净冷启动) ----------
cleanup_stale() {
  # ros2 CLI daemon 可能占住 zenoh 端口 7447, 导致 router 起不来 -> 先停掉
  timeout 5 ros2 daemon stop > /dev/null 2>&1 || true
  pkill -f "ros2.*daemon" 2>/dev/null
  for pat in "rmw_zenohd" "pc_map_pub.py" "ws_map_pub.py" "front_cam_bridge.py" "cam_view.py" "custom_nav_node" "rviz2"; do
    pkill -f "[${pat:0:1}]${pat:1}" 2>/dev/null
  done
  sleep 2
}

# ---------- 2. 退出信号处理: 受控趴下 ----------
SHUTDOWN_DONE=0
laydown_all() {
  [ "$SHUTDOWN_DONE" = "1" ] && exit 0
  SHUTDOWN_DONE=1
  echo ""
  echo "[$(now)] 收到退出信号, 收工流程: 停运动 -> 半蹲 -> 阻尼趴下"
  timeout 8 ros2 service call /set_motion_enabled std_srvs/srv/SetBool \
    "{data: false}" > /dev/null 2>&1
  if timeout 10 ros2 service call /zs1w/lie_down std_srvs/srv/Trigger \
      2>&1 | grep -qi "success"; then
    echo "  ✓ 受控半蹲完成 (lie_down)"
    sleep 2
    if timeout 10 ros2 service call /zs1w/passive std_srvs/srv/Trigger \
        2>&1 | grep -qi "success"; then
      echo "  ✓ 阻尼完全趴下 (passive)"
    else
      echo "  (passive 调用失败, 狗停在半蹲, 请手动确认安全)"
    fi
  else
    echo "  (lie_down 调用失败, 直接阻尼趴下)"
    timeout 10 ros2 service call /zs1w/passive std_srvs/srv/Trigger \
      2>&1 | grep -qi "success" && echo "  ✓ 阻尼完全趴下 (passive)"
  fi
  [ -f "$PID_FILE" ] && [ "$(cat "$PID_FILE" 2>/dev/null)" = "$$" ] && rm -f "$PID_FILE"
  echo "  清理 PC 端进程 ..."
  cleanup_stale
  echo "=== 退出 ==="
  exit 0
}
trap laydown_all INT TERM

# 守护子进程写 pid (供 --stop)
if [ "$BOOT" = "1" ]; then
  echo $$ > "$PID_FILE"
fi

# ---------- 3. 实例互斥 ----------
if [ -f "$PID_FILE" ] && [ "$BOOT" != "1" ]; then
  BPID=$(cat "$PID_FILE")
  if [ -n "$BPID" ] && [ "$BPID" != "$$" ] && kill -0 "$BPID" 2>/dev/null; then
    echo "已有一个后台守护在运行 (pid $BPID)。"
    echo "如需重启: $0 --stop  然后重新 $0 --boot"
    exit 1
  fi
fi

echo "=== [0/8] 环境就绪: domain=$ROS_DOMAIN_ID rmw=$RMW_IMPLEMENTATION ==="
if [ "$BOOT" = "1" ]; then
  echo "=== (守护模式, 前台日志见 $LOG_DIR/boot.log) ==="
fi

echo "=== [0.5/8] 清理 PC 端残留进程 ... ==="
cleanup_stale
LEFT=$(ps aux | grep -E "rmw_zenohd|pc_map_pub|ws_map_pub|front_cam_bridge|cam_view|custom_nav_node|rviz2" | grep -v grep | wc -l)
echo "      残留进程已清理 (剩余 $LEFT)"

# ---------- 4. zenoh router ----------
echo "=== [1/8] 启动 zenoh router ... ==="
setsid nohup env ZENOH_ROUTER_CONFIG_URI="$ZENOH_CONFIG" \
  /opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd \
  > "$LOG_DIR/zenohd.log" 2>&1 < /dev/null &
sleep 3
# 验证 router 真起来了(端口7447可能被 ros2 daemon 占用导致静默失败)
if ! pgrep -f "rmw_zenohd" > /dev/null 2>&1; then
  echo "  !! zenoh router 启动失败, 清理端口后重试 ..."
  pkill -f "ros2.*daemon" 2>/dev/null
  sleep 1
  setsid nohup env ZENOH_ROUTER_CONFIG_URI="$ZENOH_CONFIG" \
    /opt/ros/humble/lib/rmw_zenoh_cpp/rmw_zenohd \
    > "$LOG_DIR/zenohd.log" 2>&1 < /dev/null &
  sleep 3
  pgrep -f "rmw_zenohd" > /dev/null 2>&1 && echo "  ✓ zenoh router 已启动" || echo "  !! zenoh router 仍失败 (见 $LOG_DIR/zenohd.log)"
fi

# ---------- 5. 首次拉起: 驱动 -> 站立 -> 定位 -> /map -> nav -> rviz ----------
echo "=== [2/8] 检查板子 zs1w_driver ... ==="
ensure_driver

echo "=== [3/8] 自动站立 (stand_up) ... ==="
auto_stand
sleep 1

echo "=== [4/8] 等待 zenoh 链路就绪 ... ==="
for i in $(seq 1 10); do
  if timeout 3 ros2 topic echo /tf_manager/odom_type --once 2>/dev/null | grep -q frame_id; then
    echo "      链路就绪 (${i}s)"
    break
  fi
  [ "$i" = "10" ] && echo "  !! 无法读取 odom_type, 继续 ..."
  sleep 1
done

echo "      定位检测 ..."
if odom_is_localized; then
  echo "      定位已正常 (localization_odom)"
else
  echo "      定位未就绪 / 卡死, 执行恢复序列 ..."
  do_localization_recover
  sleep 2
  if odom_is_localized; then
    echo "      定位恢复成功: $(timeout 6 ros2 topic echo /tf_manager/odom_type --once 2>/dev/null | grep frame_id)"
  else
    echo "  !! 定位仍未恢复, 监控循环会持续重试"
  fi
fi

echo "=== [5/8] 发布 /map (ws_map_pub 直拉板上真图) ... ==="
ensure_ws_map_pub
wait_map_ready && echo "      /map 已可读取" || echo "      /map 暂不可读 (狗上线后自动恢复)"

echo "=== [6/8] 启动 custom_nav (auto_enable_on_goal) ... ==="
ensure_custom_nav

echo "=== [7/8] 启动 rviz + 相机窗口 ... ==="
ensure_rviz
# 新一次启动重新打开相机 (清掉上次"用户已关闭"标记)
rm -f "$LOG_DIR/cam_view.user_closed"
ensure_cam_view

echo "=== [8/8] 启动完成, 状态汇总 ==="
echo "  --- 进程 ---"
for p in rmw_zenohd ws_map_pub custom_nav_node cam_view rviz2; do
  pgrep -af "$p" | head -1 || true
done
echo "  --- 狗状态 ---"
timeout 6 ros2 topic echo /tf_manager/odom_type --once 2>&1 | grep frame_id || echo "  (无法读取 odom_type)"
echo "  --- nav_status ---"
timeout 6 ros2 topic echo /nav_status --once 2>&1 | grep data || echo "  (custom_nav 未输出)"

echo ""
echo "============================================================"
echo " 一键启动完成! 现在只需:"
echo "  在 rviz 用 2D Goal 点目标 -> 狗自动使能并自主导航过去"
echo ""
if [ "$BOOT" = "1" ]; then
  echo "  当前为后台守护模式 (自愈中). 停止: $0 --stop"
  echo "  查看日志: tail -f $LOG_DIR/boot.log"
else
  echo "  本脚本保持前台运行 (自愈中)."
  echo "  按 Ctrl+C 结束 -> 自动: 停运动 -> 半蹲(lie_down) -> 趴下(passive)"
fi
echo "  紧急急停(任意时刻, 前台): python3 src/custom_nav/scripts/e_stop.py [按 q]"
echo "============================================================"

# ---------- 6. 常驻自愈监控 ----------
MONITOR_INTERVAL=5
LAST_RECOVER=0
PREV_ONLINE="unknown"
while true; do
  sleep "$MONITOR_INTERVAL"

  if ! zs1w_online; then
    PREV_ONLINE=no
    # 驱动掉线: 狗重启或网络断. 板子可 ping 通才尝试拉起, 否则静默等
    if board_ping; then
      log "驱动掉线但板子可 ping 通 -> 尝试重新拉起驱动"
      ensure_driver
      auto_stand
      if zs1w_online; then
        log "驱动恢复, 执行定位恢复 ..."
        do_localization_recover
        PREV_ONLINE=yes
      fi
    fi
    # 进程守护: nav/rviz/相机窗口 保持, map_pub 会自愈(内部重连)
    ensure_custom_nav
    ensure_rviz
    ensure_cam_view
    continue
  fi

  # 驱动在线
  if [ "$PREV_ONLINE" = "no" ] || [ "$PREV_ONLINE" = "unknown" ]; then
    log "检测到驱动在线 (恢复/首次), 自动站立 + 定位确认 ..."
    auto_stand
    PREV_ONLINE=yes
  fi

  if ! odom_is_localized; then
    NOW=$(date +%s)
    if [ $((NOW - LAST_RECOVER)) -ge 20 ]; then
      LAST_RECOVER=$NOW
      log "定位卡死 / 未就绪 (odom_type!=localization_odom), 自动恢复 ..."
      do_localization_recover
      auto_stand
    fi
  fi

  # 进程守护
  ensure_ws_map_pub
  ensure_custom_nav
  ensure_rviz
  ensure_cam_view
done
