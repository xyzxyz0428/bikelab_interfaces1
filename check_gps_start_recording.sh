#!/usr/bin/env bash
set -euo pipefail

SESSION="bikelab_interfaces1"
WIN_NAME="sensors"

# --- prerequisites ---
command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found in PATH"; exit 2; }
command -v tmux >/dev/null 2>&1 || { echo "tmux not found. Install: sudo apt install -y tmux"; exit 2; }

# must run inside the existing tmux session you mentioned
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' not found. Please start it first (your previous session)."
  exit 2
fi

echo "# GPS check using /ubx_nav_pvt"
out="$(
  ros2 topic echo --once /ubx_nav_pvt 2>/dev/null \
  | awk '
  /^diff_soln:/ { d = ($2=="true") ? 1 : 0 }
  /^carr_soln:$/ { in_c=1; next }
  in_c && /^[[:space:]]+status:/ { c=$2; in_c=0 }
  END {
    c+=0; d+=0;
    printf("carr_soln.status =%d diff_soln=%d\n", c, d);
    print ((c!=0 && d==1) ? "true" : "false");
  }'
)"

status_line="$(printf "%s\n" "$out" | head -n 1)"
ok="$(printf "%s\n" "$out" | tail -n 1)"

# always show the numbers
echo "$status_line"

if [[ "$ok" != "true" ]]; then
  echo "false"
  # values already printed above; exit non-zero for scripting convenience
  exit 1
fi

echo "true"

echo
echo "# GPS ok -> start 3 sensor nodes in tmux session '$SESSION' (one window, 3 panes)"

# If window already exists, do not duplicate
if tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$WIN_NAME"; then
  echo "tmux window '$WIN_NAME' already exists in session '$SESSION'. Not creating again."
  echo "Attach: tmux attach -t $SESSION  (then switch window: Ctrl-b + n / p)"
  exit 0
fi

# Create new window
tmux new-window -t "$SESSION" -n "$WIN_NAME"

# Pane 0 (left-top): potentiometer
tmux send-keys -t "$SESSION:$WIN_NAME.0" \
  "ros2 run bikelab_interfaces potentiometer_driver.py" C-m

# Split vertically -> Pane 1 (bottom)
tmux split-window -v -t "$SESSION:$WIN_NAME"

# Pane 1 (bottom): garmin
tmux send-keys -t "$SESSION:$WIN_NAME.1" \
  "ros2 run bikelab_interfaces garmin_driver.py" C-m

# Go back to pane 0 and split horizontally -> Pane 2 (right-top)
tmux select-pane -t "$SESSION:$WIN_NAME.0"
tmux split-window -h -t "$SESSION:$WIN_NAME"

# Pane 2 (right-top): IMU node (with your args)
tmux send-keys -t "$SESSION:$WIN_NAME.2" \
  "ros2 run wheeltec_n100_imu imu_node --ros-args -p serial_port:=\"/dev/serial/by-id/usb-Dynastream_Innovations_ANT_USB-m_Stick_065-if00-port0\" -p serial_baud:=115200" C-m

# Make layout tidy (top split into 2, bottom full width)
tmux select-layout -t "$SESSION:$WIN_NAME" main-horizontal

echo "Started in tmux. Attach: tmux attach -t $SESSION"
echo "In tmux: switch panes Ctrl-b then o / arrows. Detach: Ctrl-b then d."
