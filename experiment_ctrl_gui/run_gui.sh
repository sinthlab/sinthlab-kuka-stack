#!/usr/bin/env bash
# Start the experiment control dashboard. Sources ROS 2 and the workspace first (ros2 launch and the
# live controls need both), then serves http://localhost:8080 . Extra arguments go to server.py:
#   ./run_gui.sh --port 8081        ./run_gui.sh --demo        ./run_gui.sh --help
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${LBR_WS:-$(cd "$HERE/../../.." && pwd)}"          # ~/lbr-stack by default

if [[ -z "$ROS_DISTRO" && -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
fi
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
else
  echo "Note: $WS/install/setup.bash not found -- build the workspace first (README §3), or set LBR_WS." >&2
fi
exec python3 "$HERE/server.py" "$@"
