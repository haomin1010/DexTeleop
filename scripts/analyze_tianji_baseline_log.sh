#!/usr/bin/env bash
# Summarize a sim or real teleop main.log for baseline comparison.
set -euo pipefail

LOG="${1:-}"
if [ -z "$LOG" ] || [ ! -f "$LOG" ]; then
  echo "Usage: $0 /path/to/main.log" >&2
  exit 1
fi

echo "=== Log: $LOG ==="
echo ""

echo "--- Startup profile (first match) ---"
grep -m1 'use pinocchio IK' "$LOG" || true
grep -m1 'tracker orientation input mode' "$LOG" || true
grep -m1 'READ ONLY' "$LOG" || true
grep -m1 'TRACKER_ORI_MAP' "$LOG" || true
echo ""

echo "--- TRACKER_ORI_SOURCE (last 3) ---"
grep 'TRACKER_ORI_SOURCE' "$LOG" | tail -3 || echo "(none)"
echo ""

echo "--- Counts ---"
printf "READ_ONLY_POSE:     "; grep -c 'READ_ONLY_POSE' "$LOG" 2>/dev/null || echo 0
printf "READ_ONLY_POSE ok:  "; grep 'READ_ONLY_POSE' "$LOG" | grep -c 'right_success=True' 2>/dev/null || echo 0
printf "READ_ONLY_POSE fail:"; grep 'READ_ONLY_POSE' "$LOG" | grep -c 'right_success=False' 2>/dev/null || echo 0
printf "RIGHT_IK_FAIL:      "; grep -c 'RIGHT_IK_FAIL' "$LOG" 2>/dev/null || echo 0
printf "SDK_IK hold:        "; grep -c 'SDK_IK.*holding' "$LOG" 2>/dev/null || echo 0
printf "Marvin Set B cmd:   "; grep -c 'Set B arm joint cmd' "$LOG" 2>/dev/null || echo 0
printf "TRACKER_ORI lines:  "; grep -c '\\[TRACKER_ORI\\]' "$LOG" 2>/dev/null || echo 0
echo ""

echo "--- Last READ_ONLY_POSE ---"
grep 'READ_ONLY_POSE' "$LOG" | tail -1 || echo "(none)"
echo ""

echo "--- Last RIGHT_IK_FAIL ---"
grep 'RIGHT_IK_FAIL' "$LOG" | tail -1 || echo "(none)"
echo ""

echo "--- Large orientation deltas (blended_delta_deg > 45) ---"
grep 'TRACKER_ORI' "$LOG" | grep -E 'blended_delta_deg=[4-9][0-9]\.|blended_delta_deg=[1-9][0-9]{2}' | tail -5 || echo "(none in tail)"
