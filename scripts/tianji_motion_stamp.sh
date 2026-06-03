#!/usr/bin/env bash
# Log a motion step timestamp into a sim/real baseline capture bundle.
# Run from HOST while teleop is running (second terminal).
#
# Usage:
#   ./scripts/tianji_motion_stamp.sh /tmp/tianji_sim_baseline_20260530_155325 1 zero_hold
#   ./scripts/tianji_motion_stamp.sh wuji22-hand:/tmp/tianji_sim_baseline_xxx 2 trans_x
set -euo pipefail

if [ "$#" -lt 3 ]; then
  echo "Usage: $0 <capture_dir_or_container:path> <step_id> <label...>" >&2
  echo "Example:" >&2
  echo "  $0 /tmp/tianji_sim_baseline_20260530_155325 1 zero_hold" >&2
  echo "  $0 wuji22-hand:/tmp/tianji_sim_baseline_20260530_155325 3 roll_only" >&2
  exit 1
fi

TARGET="$1"
STEP="$2"
shift 2
LABEL="$*"
LINE="$(date -Is) step=${STEP} label=${LABEL}"

if [[ "$TARGET" == *:* ]]; then
  CONTAINER="${TARGET%%:*}"
  PATH_IN_CONTAINER="${TARGET#*:}"
  docker exec "$CONTAINER" bash -lc 'mkdir -p "$1" && printf "%s\n" "$2" >> "$1/motion_log.txt"' _ "$PATH_IN_CONTAINER" "$LINE"
else
  mkdir -p "$TARGET"
  echo "$LINE" >> "$TARGET/motion_log.txt"
  echo "$LINE"
fi
