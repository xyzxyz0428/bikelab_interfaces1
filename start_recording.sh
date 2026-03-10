#!/usr/bin/env bash
set -euo pipefail

SESSION="bikelab_interfaces1"
WIN_NAME="sensors"
SCRIPT_DIR="$HOME/ros2_ws/src/bikelab_interfaces1/bikelab_interfaces/scripts"

# --- prerequisites ---
command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found in PATH"; exit 2; }
command -v tmux >/dev/null 2>&1 || { echo "tmux not found. Install: sudo apt install -y tmux"; exit 2; }

# must run inside the existing tmux session
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' not found. Please start it first."
  exit 2
fi

# check scripts directory
if [[ ! -d "$SCRIPT_DIR" ]]; then
  echo "script directory not found: $SCRIPT_DIR"
  exit 2
fi

echo "# GPS check using /ubx_nav_pvt (+ prints /ubx_esf_status fusionMode)"

out="$(
  { ros2 topic echo --once /ubx_nav_pvt; echo "__ESF__"; ros2 topic echo --once /ubx_esf_status; } 2>/dev/null \
  | awk '
    $0=="__ESF__" { mode="esf"; next }

    # --- NAV-PVT parsing (RTK status) ---
    mode!="esf" {
      if ($1=="diff_soln:") { d = ($2=="true") ? 1 : 0; next }
      if ($1=="carr_soln:") { in_c=1; next }
      if (in_c && $1=="status:") { c=$2; in_c=0; next }
      next
    }

    # --- ESF-STATUS parsing (fusionMode / fusion_mode) ---
    mode=="esf" {
      if ($1=="fusion_mode:" || $1=="fusionMode:") {
        if (NF>=2) { fm=$2; got=1; in_f=0; next }
        in_f=1; next
      }
      if (in_f && $1=="status:") { fm=$2; got=1; in_f=0; next }
      next
    }

    END {
      c += 0; d += 0

      if (!got) { fm_s="NA" }
      else { fm += 0; fm_s=fm }

      printf("carr_soln.status=%d diff_soln=%d fusion_mode.status=%s\n", c, d, fm_s)

      ok = (c != 0 && d == 1) ? "true" : "false"
      print ok
    }'
)"

status_line="$(printf "%s\n" "$out" | head -n 1)"
ok="$(printf "%s\n" "$out" | tail -n 1)"

echo "$status_line"
echo "$ok"

if [[ "$ok" != "true" ]]; then
  echo "false"
  exit 1
fi

echo "true"
echo
echo "# GPS ok -> start 4 sensor loggers in tmux session '$SESSION'"

# If window already exists, do not duplicate
if tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$WIN_NAME"; then
  echo "tmux window '$WIN_NAME' already exists in session '$SESSION'. Not creating again."
  echo "Attach: tmux attach -t $SESSION"
  exit 0
fi

# create new window and capture the first pane id
tmux new-window -t "$SESSION" -n "$WIN_NAME"
PANE_POT="$(tmux display-message -p -t "$SESSION:$WIN_NAME" '#{pane_id}')"

# split vertically from first pane -> bottom pane
PANE_POWER="$(tmux split-window -v -P -F '#{pane_id}' -t "$PANE_POT")"

# split horizontally from the original top pane -> top-right pane
PANE_IMU="$(tmux split-window -h -P -F '#{pane_id}' -t "$PANE_POT")"


# layout tidy for 4 panes
tmux select-layout -t "$SESSION:$WIN_NAME" tiled

# Pane: potentiometer logger
tmux send-keys -t "$PANE_POT" \
  "cd \"$SCRIPT_DIR\" && python3 log_potentiometer_to_csv.py" C-m

# Pane: rally power logger
tmux send-keys -t "$PANE_POWER" \
  "cd \"$SCRIPT_DIR\" && python3 ant_speed_power_logger.py --bike_speed_id 18412 --power_meter_id 64434 --out_dir ~/bikelab_interface_logs/" C-m

# Pane: IMU logger
tmux send-keys -t "$PANE_IMU" \
  "cd \"$SCRIPT_DIR\" && python3 log_imu_to_csv.py --port /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_0003-if00-port0 --baud 115200 --timeout_ms 20" C-m


# keep layout tidy
tmux select-layout -t "$SESSION:$WIN_NAME" tiled

echo "Started in tmux. Attach: tmux attach -t $SESSION"
echo "Potentiometer pane: $PANE_POT"
echo "Power pane:         $PANE_POWER"
echo "IMU pane:           $PANE_IMU"
echo "In tmux: switch panes Ctrl-b then o / arrows. Detach: Ctrl-b then d."