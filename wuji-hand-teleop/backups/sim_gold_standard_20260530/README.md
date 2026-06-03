# Sim gold standard backup (2026-05-30)

Frozen copies of the **verified sim teleop** stack before B/E keyboard gate and real-robot init changes.

## Restore (rollback code + config)

From repo root:

```bash
./wuji-hand-teleop/backups/sim_gold_standard_20260530/RESTORE.sh
```

Then rebuild in docker: `colcon build --packages-select controller tianji_output wuji_teleop_bringup`

## Sim capture launch (unchanged behavior)

Use **`tianji_output_sim_FROZEN.yaml`** or restored `tianji_output_sim.yaml` with:

```bash
read_only:=true feedback_handshake:=true
keyboard_teleop_gate:=false   # default in frozen yaml
```

Script: `scripts/run_tianji_sim_baseline_capture.sh`

## Files in this folder

| File | Role |
|------|------|
| `tianji_output_sim_FROZEN.yaml` | Do not edit — exact sim IK/tracker profile |
| `tianji_arm_node.py` | Node before keyboard gate |
| `tianji_arm_controller.py` | Controller snapshot at backup time |
| `static_transforms.yaml` | Wrist → tianji_right static TF |
| `wuji_teleop_arm.launch.py` | Arm-only launch |
| `run_tianji_sim_baseline_capture.sh` | Bundle capture script |

## Real robot (new profile, not this folder)

Use `tianji_output_real_teleop.yaml` after restore-safe changes: `keyboard_teleop_gate: true`, `init_move_sides: right`, B/E + 3s align on B.
