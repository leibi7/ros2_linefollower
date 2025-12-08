#!/usr/bin/env bash
set -euo pipefail
export DISPLAY=${DISPLAY:-:1}

# Wait for gzclient window then maximize to fill the framebuffer.
for i in {1..50}; do
  wid=$(xdotool search --onlyvisible --name "Gazebo" || true)
  if [ -n "$wid" ]; then
    # Pick first window id
    wid=$(echo "$wid" | head -n1)
    xdotool windowactivate "$wid"
    wmctrl -ir "$wid" -b add,maximized_vert,maximized_horz || true
    xdotool windowsize "$wid" 1680 1050
    xdotool windowmove "$wid" 0 0
    exit 0
  fi
  sleep 0.2
done
exit 0
