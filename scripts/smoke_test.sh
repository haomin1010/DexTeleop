#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

"$PYTHON_BIN" - <<'PY'
from pathlib import Path
from dexproj.session.run_session import SessionConfig, _load_plan, _resolve_configs

root = Path('/home/user/workspace/DexProj')
config_path = root / 'config' / 'session_htc_wuji_glove.yaml'
session_cfg, bringup_cfg, hand_cfg, bringup_command = _resolve_configs(config_path)
plan = _load_plan(session_cfg, bringup_command, hand_cfg, bringup_cfg)
assert session_cfg.enable_camera is True
assert session_cfg.camera_config.endswith('camera_config.yaml')
assert plan['camera']['enabled'] is True
assert plan['camera']['config'].endswith('camera_config.yaml')
assert plan['mode'] in {'single_left', 'single_right', 'dual'}
assert len(plan['hand_teleop']) > 0
print('config_ok')
print('bringup_cmd:', ' '.join(bringup_command))
PY

echo "smoke_ok"
