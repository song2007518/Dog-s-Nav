#!/usr/bin/env bash
set -e
CONSOLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR="$CONSOLE_DIR/runtime/ros_logs"
if [[ "${1:-}" == "--robot" ]]; then
  source "$CONSOLE_DIR/robot_env.sh"
fi
exec /usr/bin/python3 "$CONSOLE_DIR/tools/collect_diagnostics.py" "$@"
