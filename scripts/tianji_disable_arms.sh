#!/usr/bin/env bash
set -euo pipefail

ROBOT_IP="${TIANJI_ROBOT_IP:-192.168.8.166}"

if [[ -f /workspace/DexProj/scripts/activate_dexproj_env.sh ]]; then
    source /workspace/DexProj/scripts/activate_dexproj_env.sh
elif [[ -f /home/user/workspace/DexProj/scripts/activate_dexproj_env.sh ]]; then
    source /home/user/workspace/DexProj/scripts/activate_dexproj_env.sh
fi

python3 -u - "$ROBOT_IP" <<'PY'
import sys
import time

from tianji_output._internal.fx_robot import Marvin_Robot

robot_ip = sys.argv[1]
robot = Marvin_Robot()
ok = robot.connect(robot_ip)
print(f"[tianji-disable] connect {robot_ip}: {ok}")
if not ok:
    raise SystemExit(2)

try:
    robot.clear_set()
    robot.set_state(arm="A", state=0)
    robot.set_state(arm="B", state=0)
    robot.send_cmd()
    print("[tianji-disable] sent A/B state=0")
    time.sleep(0.5)
finally:
    robot.release_robot()
    print("[tianji-disable] released robot")
PY
