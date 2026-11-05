# Sourced by the launch and diagnostic scripts. Uses a per-process direct connection.
export ROS_DOMAIN_ID="${ROBOT_CONSOLE_DOMAIN_ID:-24}"
export RMW_IMPLEMENTATION="${ROBOT_CONSOLE_RMW:-rmw_zenoh_cpp}"
unset ROS_LOCALHOST_ONLY
if [[ "$RMW_IMPLEMENTATION" == "rmw_zenoh_cpp" && -z "${ZENOH_SESSION_CONFIG_URI:-}" ]]; then
  export ZENOH_SESSION_CONFIG_URI="$CONSOLE_DIR/runtime/direct_session.json5"
  mkdir -p "$CONSOLE_DIR/runtime"
  /usr/bin/python3 - <<'PY'
import os
from pathlib import Path
from ipaddress import ip_address
host=str(ip_address(os.environ.get('ROBOT_CONSOLE_ROUTER','192.168.234.1')))
source=Path('/opt/ros/humble/share/rmw_zenoh_cpp/config/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5').read_text()
Path(os.environ['ZENOH_SESSION_CONFIG_URI']).write_text(source.replace('tcp/localhost:7447','tcp/'+host+':7447'))
PY
fi
