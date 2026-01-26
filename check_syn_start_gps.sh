#!/usr/bin/env bash
set -euo pipefail

echo "# 1) NTP sync check (via hotspot internet)"
echo "#    true= system clock is synced by NTP"
echo "#    false= not synced / unknown"
timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -qi "^yes$" && echo true || echo false

echo
echo "# 2) PTP GM check (pmc GET PORT_DATA_SET)"
echo "#    true  = portState MASTER (this computer is acting as PTP Grandmaster)"
echo "#    false = not MASTER (this computer is not the GM)"
sudo pmc -u -b 0 "GET PORT_DATA_SET" 2>/dev/null | grep -q "portState[[:space:]]*MASTER" && echo true || echo false

echo
echo "# 3) Start GPS rover + NTRIP client in ONE tmux window (split panes)"
echo "#    - creates tmux session: bikelab_interfaces1"
echo "#    Attach with: tmux attach -t bikelab_interfaces1"
echo "#    Switch pane: Ctrl-b then Up/Down (or o)"
echo "#    Stop node: Ctrl-c in that pane"
echo "#    Detach (keep running): Ctrl-b then d"

command -v tmux >/dev/null 2>&1 || {
  echo "tmux not found. Install it with: sudo apt update && sudo apt install -y tmux"
  exit 1
}

SESSION="bikelab_interfaces1"

# If session exists, don't duplicate
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "# tmux session '$SESSION' already exists."
  echo "# Attach with: tmux attach -t $SESSION"
  exit 0
fi

# Create session (one window)
tmux new-session -d -s "$SESSION" -n gps

# Top pane: rover
tmux send-keys -t "$SESSION:0.0" \
  "echo '[GPS] Starting ublox rover (hpposllh)...'; ros2 launch ublox_dgnss ublox_rover_hpposllh.launch.py" C-m

# Split vertically to create bottom pane for ntrip
tmux split-window -v -t "$SESSION:0"

# Bottom pane: ntrip
tmux send-keys -t "$SESSION:0.1" \
  "echo '[NTRIP] Starting NTRIP client...'; ros2 launch ublox_dgnss ntrip_client_cli.launch.py" C-m

# Optional: make panes equal size
tmux select-layout -t "$SESSION:0" even-vertical

echo "# Started. Attach now:"
echo "tmux attach -t $SESSION"
