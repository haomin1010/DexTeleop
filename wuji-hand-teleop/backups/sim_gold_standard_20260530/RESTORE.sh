#!/usr/bin/env bash
# Restore sim gold-standard files from this backup directory.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
WT="$ROOT/wuji-hand-teleop"

echo "Restoring sim gold standard from $HERE"
cp -a "$HERE/tianji_output_sim_FROZEN.yaml" \
  "$WT/src/output_devices/tianji_output/tianji_output/config/tianji_output_sim.yaml"
cp -a "$HERE/tianji_arm_node.py" "$WT/src/controller/controller/tianji_arm_node.py"
cp -a "$HERE/tianji_arm_controller.py" \
  "$WT/src/output_devices/tianji_output/tianji_output/tianji_arm_controller.py"
cp -a "$HERE/static_transforms.yaml" "$WT/src/wuji_teleop_bringup/config/static_transforms.yaml"
cp -a "$HERE/wuji_teleop_arm.launch.py" "$WT/src/wuji_teleop_bringup/launch/wuji_teleop_arm.launch.py"
cp -a "$HERE/run_tianji_sim_baseline_capture.sh" "$ROOT/scripts/run_tianji_sim_baseline_capture.sh"
chmod +x "$ROOT/scripts/run_tianji_sim_baseline_capture.sh"
echo "Done. Rebuild: colcon build --packages-select controller tianji_output wuji_teleop_bringup"
