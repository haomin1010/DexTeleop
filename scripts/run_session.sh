#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
    CONTAINER_WORKDIR="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"

    sync_recordings_back() {
        mkdir -p "$ROOT_DIR/data"
        if docker cp "$CONTAINER_NAME:$CONTAINER_WORKDIR/data/." "$ROOT_DIR/data" >/dev/null 2>&1; then
            echo "[dexproj] synced container data back to host: $ROOT_DIR/data"
        else
            echo "[dexproj] warning: failed to sync container data back to host." >&2
        fi
    }

    set +e
    "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/run_session.sh" "$@"
    status=$?
    set -e
    sync_recordings_back
    exit "$status"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"

python3 -m dexproj.session "$@"
