#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [ "${DEXPROJ_USE_DOCKER:-0}" = "1" ] && [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/check_devices.sh" "$@"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"

python3 -m dexproj.check_devices "$@"
