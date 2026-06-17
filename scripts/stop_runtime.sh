#!/usr/bin/env bash
set -euo pipefail

PATTERN='/usr/bin/wuji-studio|runtime/teleop_bridge.py|hand_zenoh_bridge|teleop_real.py|wuji_glove_input|wujihand_controller|wujihand_driver_node|tianji_arm_controller|tianji_sdk_executor|ros2 launch wuji_teleop|wuji_teleop.launch|wuji_teleop_hand.launch|wuji_teleop_single.launch'

echo "[dexproj] stopping Wuji/DexProj runtime processes..."

mapfile -t pids < <(pgrep -f "$PATTERN" | grep -v "^$$$" || true)
if [ "${#pids[@]}" -eq 0 ]; then
    echo "[dexproj] no matching runtime processes found."
    exit 0
fi

kill "${pids[@]}" 2>/dev/null || true
sleep 1

mapfile -t remaining < <(pgrep -f "$PATTERN" | grep -v "^$$$" || true)
if [ "${#remaining[@]}" -gt 0 ]; then
    kill -9 "${remaining[@]}" 2>/dev/null || true
fi

echo "[dexproj] runtime processes stopped."
