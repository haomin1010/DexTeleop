#!/usr/bin/env bash
# Real robot: BOTH arms.
# Usage: ./scripts/run_tianji_real_teleop_both.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_DEXPROJ="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
CONTROLLER_CONFIG="${TIANJI_REAL_BOTH_CONFIG:-$CONTAINER_DEXPROJ/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output_real_teleop_both.yaml}"
ENABLE_MUJOCO="${ENABLE_MUJOCO:-false}"

MUJOCO_ARGS=""
if [ "$ENABLE_MUJOCO" = "true" ] || [ "$ENABLE_MUJOCO" = "1" ]; then
  MUJOCO_ARGS="enable_mujoco:=true mujoco_side:=right"
fi

exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" -- bash -lc "
set -euo pipefail
source $CONTAINER_DEXPROJ/scripts/activate_dexproj_env.sh
cd /home/wuji/ros2_ws
colcon build --packages-select openvr_input tianji_output tianji_urdf controller wuji_teleop_bringup --symlink-install 2>&1 | tail -15
set +u
source install/setup.bash
set -u
ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py \\
  arm_input:=tracker \\
  dry_run:=true \\
  read_only:=false \\
  feedback_handshake:=false \\
  sdk_executor_enable:=true \\
  sim_viz:=true \\
  enable_rviz:=true \\
  $MUJOCO_ARGS \\
  controller_config:=$CONTROLLER_CONFIG
"
