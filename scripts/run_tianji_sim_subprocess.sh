#!/usr/bin/env bash
# Sim harness: read_only + MuJoCo/RViz (default: in-process IK, no subprocess isolate).
#
# Usage:
#   ./scripts/run_tianji_sim_subprocess.sh
#
# Optional:
#   ENABLE_MUJOCO=false ./scripts/run_tianji_sim_subprocess.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_DEXPROJ="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
CONTROLLER_CONFIG="${TIANJI_SIM_SUBPROCESS_CONFIG:-$CONTAINER_DEXPROJ/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output_sim_subprocess.yaml}"
ENABLE_MUJOCO="${ENABLE_MUJOCO:-true}"

MUJOCO_ARGS=""
if [ "$ENABLE_MUJOCO" = "true" ] || [ "$ENABLE_MUJOCO" = "1" ]; then
  MUJOCO_ARGS="enable_mujoco:=true"
fi

exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" -- bash -lc "
set -euo pipefail
source $CONTAINER_DEXPROJ/scripts/activate_dexproj_env.sh
cd /home/wuji/ros2_ws
colcon build --packages-select openvr_input wujihand_output tianji_output controller wuji_teleop_bringup --symlink-install 2>&1 | tail -15
set +u
source install/setup.bash
set -u
ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py \\
  arm_input:=tracker \\
  read_only:=true \\
  feedback_handshake:=true \\
  sim_viz:=true \\
  enable_rviz:=true \\
  $MUJOCO_ARGS \\
  controller_config:=$CONTROLLER_CONFIG
"
