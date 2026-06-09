#!/usr/bin/env bash
# Sim baseline capture: NO controller logic changes.
# Records rosbag + topic CSV + config snapshot + main log for later real-robot comparison.
#
# Usage (host):
#   ./scripts/run_tianji_sim_baseline_capture.sh
#
# Second terminal (log each motion step with timestamps):
#   ./scripts/tianji_motion_stamp.sh wuji22-hand:/tmp/tianji_sim_baseline_<stamp> 1 zero_hold
#   ./scripts/tianji_motion_stamp.sh wuji22-hand:/tmp/tianji_sim_baseline_<stamp> 2 trans_x
#   ... (see printed checklist when capture starts)
#
# After capture: fill motion_notes.txt from motion_script.txt in the bundle.
#
# Output (container): /tmp/tianji_sim_baseline_<stamp>/
# Copy to host:
#   docker cp wuji22-hand:/tmp/tianji_sim_baseline_<stamp> ./captures/
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_DEXPROJ="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
CONTROLLER_CONFIG="${TIANJI_SIM_CONTROLLER_CONFIG:-$CONTAINER_DEXPROJ/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output_sim.yaml}"
SIM_BACKUP_DIR="$CONTAINER_DEXPROJ/wuji-hand-teleop/backups/sim_gold_standard_20260530"
OPENVR_CONFIG="${OPENVR_CONFIG:-$CONTAINER_DEXPROJ/wuji-hand-teleop/src/input_devices/openvr_input/config/openvr_input.yaml}"
OBSERVER_START_DELAY_SEC="${OBSERVER_START_DELAY_SEC:-25}"
CAPTURE_DURATION_SEC="${CAPTURE_DURATION_SEC:-0}"

exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" -- bash -lc "
set -euo pipefail
OBSERVER_START_DELAY_SEC=${OBSERVER_START_DELAY_SEC}
CAPTURE_DURATION_SEC=${CAPTURE_DURATION_SEC}
source $CONTAINER_DEXPROJ/scripts/activate_dexproj_env.sh
cd /home/wuji/ros2_ws
colcon build --packages-select openvr_input wujihand_output tianji_output controller wuji_teleop_bringup --symlink-install 2>&1 | tail -20
set +u
source install/setup.bash
set -u

STAMP=\$(date +%Y%m%d_%H%M%S)
CAPTURE_DIR=/tmp/tianji_sim_baseline_\${STAMP}
mkdir -p \"\$CAPTURE_DIR\"/config_snapshot \"\$CAPTURE_DIR\"/topics \"\$CAPTURE_DIR\"/observers

MAIN_LOG=\"\$CAPTURE_DIR/main.log\"
README=\"\$CAPTURE_DIR/README.txt\"

cat > \"\$README\" <<'EOF'
Tianji SIM baseline capture (reference for real robot)
====================================================

This bundle is the gold standard: read_only + TJ SDK IK + mapped_tf.
Do NOT change tianji_arm_node logic when comparing; fix real-robot config/path only.

Rollback sim stack: wuji-hand-teleop/backups/sim_gold_standard_20260530/RESTORE.sh
Frozen yaml (no gate): tianji_output_sim_FROZEN.yaml in that folder.

Files:
  main.log              - full ros2 launch stdout/stderr
  config_snapshot/      - yaml + static TF + git state at capture time
  baseline.bag/         - rosbag2: /tf, joint_command/state, ee_pose, zsp_para
  topics/*.csv          - parallel CSV echoes (joint + EE pose)
  observers/
    debug_arm_axis.log  - chest->arm / chest->tianji_right (3s)
    tf_sample.log       - periodic tf2_echo snapshots
    ros2_graph.txt      - node/topic list after teleop up

Real robot later should match:
  - Same controller_config (tianji_output_sim.yaml) except read_only:false
  - Same static_transforms.yaml
  - Compare joint_command vs this bag's /tianji_arm/right/joint_command
  - Compare [READ_ONLY_POSE] right_target_xyzabc vs real Set B cmd / FK

Motion script: motion_script.txt (in this bundle)
Motion log:    motion_log.txt   (stamp steps from host: scripts/tianji_motion_stamp.sh)
Analyze log:   ./scripts/analyze_tianji_baseline_log.sh \$MAIN_LOG
EOF

# Motion checklist in bundle
cp $CONTAINER_DEXPROJ/scripts/tianji_sim_baseline_motion_template.txt \"\$CAPTURE_DIR/motion_script.txt\"
sed -i \"s|___________|_\${STAMP}|g\" \"\$CAPTURE_DIR/motion_script.txt\" 2>/dev/null || true
: > \"\$CAPTURE_DIR/motion_log.txt\"
echo \"# motion_log: one line per step (host: ./scripts/tianji_motion_stamp.sh)\" >> \"\$CAPTURE_DIR/motion_log.txt\"

echo \"Capture dir: \$CAPTURE_DIR\"
echo \"Main log:    \$MAIN_LOG\"
cat \"\$README\"

echo ''
echo '================================================================'
echo ' MOTION CAPTURE — use a SECOND terminal while this runs'
echo '================================================================'
echo \"  CAPTURE=wuji22-hand:\$CAPTURE_DIR\"
echo ''
echo '  Sim uses keyboard_teleop_gate:=false — wait for \"Tracker zero initialized\" after ~2s delay,'
echo '  then ~3s hold + ~5–8s move each (pitch/yaw/roll small then large):'
echo '    ./scripts/tianji_motion_stamp.sh wuji22-hand:'\"\$CAPTURE_DIR\"' 1 pitch_small'
echo '    ./scripts/tianji_motion_stamp.sh wuji22-hand:'\"\$CAPTURE_DIR\"' 2 pitch_large'
echo '    ./scripts/tianji_motion_stamp.sh wuji22-hand:'\"\$CAPTURE_DIR\"' 3 yaw_small'
echo '    ./scripts/tianji_motion_stamp.sh wuji22-hand:'\"\$CAPTURE_DIR\"' 4 yaw_large'
echo '    ./scripts/tianji_motion_stamp.sh wuji22-hand:'\"\$CAPTURE_DIR\"' 5 roll_small'
echo '    ./scripts/tianji_motion_stamp.sh wuji22-hand:'\"\$CAPTURE_DIR\"' 6 roll_large'
echo ''
echo '  When done (~90–120s), Ctrl+C this terminal. Then copy bundle and edit motion_notes.txt.'
echo '================================================================'
echo ''

# --- config snapshot ---
cp -a \
  $CONTAINER_DEXPROJ/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output.yaml \
  $CONTAINER_DEXPROJ/wuji-hand-teleop/src/output_devices/tianji_output/tianji_output/config/tianji_output_sim.yaml \
  $SIM_BACKUP_DIR/tianji_output_sim_FROZEN.yaml \
  $CONTAINER_DEXPROJ/wuji-hand-teleop/src/wuji_teleop_bringup/config/static_transforms.yaml \
  $CONTAINER_DEXPROJ/wuji-hand-teleop/src/input_devices/openvr_input/config/openvr_input.yaml \
  \"\$CAPTURE_DIR/config_snapshot/\" 2>/dev/null || true
cp -a \"$SIM_BACKUP_DIR/README.md\" \"\$CAPTURE_DIR/config_snapshot/sim_backup_README.md\" 2>/dev/null || true
(
  cd $CONTAINER_DEXPROJ/wuji-hand-teleop && git rev-parse HEAD 2>/dev/null && git status -sb 2>/dev/null
  cd $CONTAINER_DEXPROJ && git rev-parse HEAD 2>/dev/null && git status -sb 2>/dev/null
) > \"\$CAPTURE_DIR/config_snapshot/git_state.txt\" 2>&1 || true

PIDS=()
cleanup() {
  for pid in \"\${PIDS[@]:-}\"; do
    kill \"\$pid\" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_observers() {
  echo \"[\$CAPTURE_DIR] Starting observers (delay \${OBSERVER_START_DELAY_SEC}s)...\"
  sleep \"\$OBSERVER_START_DELAY_SEC\"

  ros2 run tianji_output debug_arm_axis >\"\$CAPTURE_DIR/observers/debug_arm_axis.log\" 2>&1 &
  PIDS+=(\$!)

  (
    while true; do
      date -Is
      echo '--- tf2_echo right_chest right_wrist ---'
      timeout 2 ros2 run tf2_ros tf2_echo right_chest right_wrist 2>/dev/null | head -20 || echo '(unavailable)'
      echo '--- tf2_echo right_chest tianji_right ---'
      timeout 2 ros2 run tf2_ros tf2_echo right_chest tianji_right 2>/dev/null | head -20 || echo '(unavailable)'
      echo '--- tf2_echo right_chest right_arm ---'
      timeout 2 ros2 run tf2_ros tf2_echo right_chest right_arm 2>/dev/null | head -20 || echo '(unavailable)'
      sleep 3
    done
  ) >\"\$CAPTURE_DIR/observers/tf_sample.log\" 2>&1 &
  PIDS+=(\$!)

  ros2 bag record -o \"\$CAPTURE_DIR/baseline\" \
    /tf /tf_static \
    /tianji_arm/right/joint_command \
    /tianji_arm/right/joint_state \
    /tianji_arm/right/right_ee_pose \
    /tianji_arm/right/right_zsp_para \
    >\"\$CAPTURE_DIR/observers/rosbag_record.log\" 2>&1 &
  PIDS+=(\$!)

  ros2 topic echo /tianji_arm/right/joint_command --csv \
    >\"\$CAPTURE_DIR/topics/right_joint_command.csv\" 2>&1 &
  PIDS+=(\$!)

  ros2 topic echo /tianji_arm/right/joint_state --csv \
    >\"\$CAPTURE_DIR/topics/right_joint_state.csv\" 2>&1 &
  PIDS+=(\$!)

  ros2 topic echo /tianji_arm/right/right_ee_pose --csv \
    >\"\$CAPTURE_DIR/topics/right_ee_pose.csv\" 2>&1 &
  PIDS+=(\$!)

  ros2 topic echo /tianji_arm/right/right_zsp_para --csv \
    >\"\$CAPTURE_DIR/topics/right_zsp_para.csv\" 2>&1 &
  PIDS+=(\$!)

  {
    echo '=== ros2 node list ==='
    ros2 node list 2>/dev/null || true
    echo ''
    echo '=== ros2 topic list ==='
    ros2 topic list 2>/dev/null || true
  } >\"\$CAPTURE_DIR/observers/ros2_graph.txt\" 2>&1
}

start_observers &
OBSERVER_PID=\$!

echo \"[\$CAPTURE_DIR] Launching teleop (tee -> main.log). Ctrl+C when motion set is done.\"
echo \"Motion script in bundle: \$CAPTURE_DIR/motion_script.txt\"

LAUNCH_CMD=\"ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py \\
  arm_input:=tracker \\
  read_only:=true \\
  feedback_handshake:=true \\
  sim_viz:=true \\
  enable_rviz:=true \\
  enable_mujoco:=true \\
  controller_config:=$CONTROLLER_CONFIG \\
  openvr_config:=$OPENVR_CONFIG\"

if [ \"\$CAPTURE_DURATION_SEC\" -gt 0 ] 2>/dev/null; then
  timeout \"\$CAPTURE_DURATION_SEC\" \$LAUNCH_CMD 2>&1 | tee \"\$MAIN_LOG\" || true
else
  \$LAUNCH_CMD 2>&1 | tee \"\$MAIN_LOG\" || true
fi

kill \"\$OBSERVER_PID\" 2>/dev/null || true
wait \"\$OBSERVER_PID\" 2>/dev/null || true
cleanup

echo ''
echo '=== Capture complete ==='
echo \"Bundle: \$CAPTURE_DIR\"
ls -la \"\$CAPTURE_DIR\"
du -sh \"\$CAPTURE_DIR\"/* 2>/dev/null || true
echo ''
echo 'Copy to host:'
echo \"  docker cp \${DEXPROJ_CONTAINER_NAME:-wuji22-hand}:\$CAPTURE_DIR ./captures/\"
echo ''
echo 'Fill motion notes (copy motion_script.txt -> motion_notes.txt and edit Actual: fields)'
echo \"  Or rely on motion_log.txt timestamps + rosbag for alignment\"
echo ''
echo 'Quick log summary:'
$CONTAINER_DEXPROJ/scripts/analyze_tianji_baseline_log.sh \"\$MAIN_LOG\" || true
"
