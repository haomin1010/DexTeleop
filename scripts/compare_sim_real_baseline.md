# Sim baseline vs real robot — where problems usually are

This doc explains **why sim can look correct while real robot does not**, without changing code yet.

## What sim does (your verified log)

```
OpenVR TF
  → incremental + mapped_tf (tianji_right for pos AND ori)
  → Marvin SDK IK (move_to_matrix_direct)
  → read_only: publish /tianji_arm/right/joint_command only
  → MuJoCo / RViz display joint_command
```

Log signatures:

- `use pinocchio IK: False`
- `tracker orientation input mode: mapped_tf`
- `[TRACKER_ORI_SOURCE] position_tf=tianji_right orientation_tf=tianji_right`
- `[READ_ONLY_POSE] right_success=True` (most frames)

## What real robot adds (failure layers)

| Layer | Sim | Real robot risk |
|-------|-----|-----------------|
| `read_only` | true — no `Set B arm joint cmd` | false — must send SDK commands |
| IK path | SDK matrix incremental | Same only if same yaml + node; early `_teleop_control_sdk_official` bypass broke incremental |
| Feedback | Uses robot `joint_state` for FK zero | Same, but bad IK → hold → arm frozen |
| Orientation | `mapped_tf` + static TF `[0.5,-0.5,-0.5,0.5]` | `raw_wrist` or +90° static test → ~90° offset |
| IK robustness | Fail → hold in log only | Fail → no motion, looks "dead" |
| Process | exit -11 on Ctrl+C sometimes | SDK segfault killed node (-11) during IK |

**The sim baseline is the target joint trajectory** in `/tianji_arm/right/joint_command` and the target pose in `[READ_ONLY_POSE] right_target_xyzabc_mm_deg=...`.

Real robot should reproduce **the same joint_command stream** (or same target pose + successful IK), not re-tune orientation until sim bundle is matched.

## Capture workflow

1. Run `./scripts/run_tianji_sim_baseline_capture.sh`
2. Follow `scripts/tianji_sim_baseline_motion_template.txt`
3. `docker cp` bundle to `./captures/`
4. Later real run: same `tianji_output_sim.yaml` but `read_only:=false`, record same bundle name pattern
5. Diff:
   - `main.log` counts via `analyze_tianji_baseline_log.sh`
   - `topics/right_joint_command.csv` (sim vs real)
   - `config_snapshot/` must match except `read_only`

## Files in sim baseline bundle

- `baseline/` — rosbag2 (tf + joints + ee_pose + zsp)
- `topics/*.csv` — high-rate echoes
- `config_snapshot/` — frozen yaml + static_transforms
- `main.log` — controller printf including TRACKER_ORI / READ_ONLY_POSE
- `observers/debug_arm_axis.log` — TF sanity in chest frame
