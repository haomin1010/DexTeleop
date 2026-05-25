#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/run_session.sh" "$@"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"

python3 -m dexproj.session "$@"
