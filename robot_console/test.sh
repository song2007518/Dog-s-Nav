#!/usr/bin/env bash
set -e
CONSOLE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$CONSOLE_DIR/package${PYTHONPATH:+:$PYTHONPATH}"
export QT_QPA_PLATFORM=offscreen
exec /usr/bin/python3 -m unittest discover -s "$CONSOLE_DIR/package/tests" -v
