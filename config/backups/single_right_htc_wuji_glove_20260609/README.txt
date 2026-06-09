Backup of DexProj session configs before dual (both arms + both hands) upgrade.

Date: 2026-06-09
Previous mode: single_right (right Tianji arm + right Wuji hand + camera recording)

Restore:
  cp config/backups/single_right_htc_wuji_glove_20260609/*.yaml config/

Files:
  session_htc_wuji_glove.yaml  - session runner entry (mode, camera, trigger)
  bringup_htc.yaml             - ros2 launch mapping (arm controller config, sdk split)
  hand_teleop_wuji_glove.yaml  - glove/hand SN and retarget paths
