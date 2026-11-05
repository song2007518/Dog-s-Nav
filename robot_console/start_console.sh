#!/usr/bin/env bash
set -e
CONSOLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
export ROS_LOG_DIR="$CONSOLE_DIR/runtime/ros_logs"
export XDG_CONFIG_HOME="$CONSOLE_DIR/runtime/config"
mkdir -p "$ROS_LOG_DIR" "$XDG_CONFIG_HOME"
export PYTHONPATH="$CONSOLE_DIR/package${PYTHONPATH:+:$PYTHONPATH}"
CONSOLE_ROBOT=0
CONSOLE_APP=0
CONSOLE_STACK=0
unset ROBOT_CONSOLE_PRESENTATION
unset ROBOT_CONSOLE_STACK
for CONSOLE_ARG in "$@"; do
  case "$CONSOLE_ARG" in
    --robot) CONSOLE_ROBOT=1 ;;
    --stack) CONSOLE_ROBOT=1; CONSOLE_STACK=1 ;;
    --app) CONSOLE_APP=1 ;;
    --presentation) export ROBOT_CONSOLE_PRESENTATION=1 ;;
    *) echo "用法：$0 [--robot | --stack] [--app] [--presentation]"; exit 2 ;;
  esac
done
if [[ "${ROBOT_CONSOLE_PRESENTATION:-}" == 1 && -z "${QT_SCALE_FACTOR:-}" ]]; then
  export QT_SCALE_FACTOR="${ROBOT_CONSOLE_UI_SCALE:-$(/usr/bin/python3 "$CONSOLE_DIR/tools/ui_scale.py")}"
fi
if [[ "$CONSOLE_ROBOT" == 1 ]]; then
  source "$CONSOLE_DIR/robot_env.sh"
  export ROBOT_CONSOLE_CONNECT_READONLY=1
  if [[ "$CONSOLE_STACK" == 1 ]]; then
    unset ROBOT_CONSOLE_CONNECT_READONLY
    export ROBOT_CONSOLE_STACK=1
  fi
else
  unset ROBOT_CONSOLE_CONNECT_READONLY
  export ROS_DOMAIN_ID=124
  export ROS_LOCALHOST_ONLY=1
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi
if [[ "$CONSOLE_APP" == 1 ]]; then
  exec /usr/bin/python3 -m rqt_robot_console.app
fi
if [[ ! -f "$CONSOLE_DIR/install/setup.bash" ]]; then
  echo '尚未构建 rqt 插件，使用独立窗口。可运行 build.sh 构建。'
  exec /usr/bin/python3 -m rqt_robot_console.app
fi
source "$CONSOLE_DIR/install/setup.bash"
export PYTHONPATH="$CONSOLE_DIR/package${PYTHONPATH:+:$PYTHONPATH}"
exec rqt --force-discover --standalone rqt_robot_console/RobotConsole
