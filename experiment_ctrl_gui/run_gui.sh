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
# The StarTech serial adapter (visual cues over serial): keep it attached to WSL. usbipd-win's
# --auto-attach attaches it now and again after every replug, for as long as WSL runs; started here, it
# also comes back after every WSL restart. Needs the one-time `usbipd bind --hardware-id` (as
# administrator, in_electric_flange_data_path/README.md §5); attach itself needs no administrator.
USB_SERIAL_HWID="${USB_SERIAL_HWID:-0403:6001}"       # the StarTech's VID:PID in `usbipd list`
if [[ " $* " != *" --demo "* ]] && command -v usbipd.exe >/dev/null 2>&1; then
  if ! pgrep -f "usbipd.exe attach --wsl --hardware-id" >/dev/null; then
    nohup usbipd.exe attach --wsl --hardware-id "$USB_SERIAL_HWID" --auto-attach \
      >"$HERE/logs/usbipd_attach.log" 2>&1 &
  fi
  for _ in $(seq 1 20); do ls /dev/ttyUSB* >/dev/null 2>&1 && break; sleep 0.25; done
  if ls /dev/ttyUSB* >/dev/null 2>&1; then
    echo "USB serial adapter attached: $(ls /dev/ttyUSB* | tr '\n' ' ')"
  else
    echo "Note: the USB serial adapter ($USB_SERIAL_HWID) is not attached -- see $HERE/logs/usbipd_attach.log." >&2
  fi
fi

exec python3 "$HERE/server.py" "$@"
