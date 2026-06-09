#!/usr/bin/env bash
# Left-arm sim (mirror of right gold standard): Tracker TF + SDK IK + RViz.
# read_only: connects to robot feedback for FK zero, never sends motion.
#
# Usage:
#   ./scripts/run_tianji_sim_left.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_DEXPROJ="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
CONTROLLER_CONFIG="${TIANJI_SIM_LEFT_CONFIG:-$CONTAINER_DEXPROJ/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output_sim_left.yaml}"

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
  dry_run:=false \\
  sdk_executor_enable:=false \\
  read_only:=true \\
  feedback_handshake:=true \\
  sim_viz:=true \\
  enable_rviz:=true \\
  enable_mujoco:=true \\
  mujoco_side:=left \\
  controller_config:=$CONTROLLER_CONFIG
"
