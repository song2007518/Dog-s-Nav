#!/usr/bin/env bash
set -e
CONSOLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
cd "$CONSOLE_DIR"
colcon --log-base "$CONSOLE_DIR/log" build --base-paths "$CONSOLE_DIR/package" \
  --build-base "$CONSOLE_DIR/build" --install-base "$CONSOLE_DIR/install" \
  --packages-select rqt_robot_console
/usr/bin/python3 "$CONSOLE_DIR/tools/create_launchers.py"
