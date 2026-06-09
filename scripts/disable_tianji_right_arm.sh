#!/usr/bin/env bash
set -euo pipefail

cd /home/user/workspace/DexProj

./scripts/ensure_docker_exec.sh -- bash -lc '
source /workspace/DexProj/scripts/activate_dexproj_env.sh
python3 - <<'"'"'PY'"'"'
import time

from tianji_output._internal.fx_robot import Marvin_Robot

robot_ip = "192.168.8.166"
robot = Marvin_Robot()
ok = robot.connect(robot_ip)
print(f"[disable] connect {robot_ip}: {ok}")
if not ok:
    raise SystemExit(2)

try:
    robot.clear_set()
    robot.set_state(arm="B", state=0)
    robot.send_cmd()
    print("[disable] sent B arm state=0")
    time.sleep(0.5)
finally:
    robot.release_robot()
    print("[disable] released robot")
PY
'
