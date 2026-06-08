#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [ "${DEXPROJ_USE_DOCKER:-0}" = "1" ] && [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
    CONTAINER_WORKDIR="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
    HOST_EPISODE_SNAPSHOT="$(mktemp)"
    CONTAINER_EPISODE_SNAPSHOT="$(mktemp)"

    cleanup_sync_state() {
        rm -f "$HOST_EPISODE_SNAPSHOT" "$CONTAINER_EPISODE_SNAPSHOT"
    }

    trap cleanup_sync_state EXIT

    snapshot_host_episodes() {
        mkdir -p "$ROOT_DIR/data/raw"
        find "$ROOT_DIR/data/raw" \
            -mindepth 2 \
            -maxdepth 2 \
            -type d \
            -name 'episode_*' \
            -printf '%P\n' \
            | sort > "$HOST_EPISODE_SNAPSHOT"
    }

    sync_recordings_back() {
        mkdir -p "$ROOT_DIR/data/raw"

        if ! docker exec "$CONTAINER_NAME" bash -lc \
            "cd $(printf '%q' "$CONTAINER_WORKDIR/data/raw") && find . -mindepth 2 -maxdepth 2 -type d -name 'episode_*' | sed 's#^./##' | sort" \
            > "$CONTAINER_EPISODE_SNAPSHOT"; then
            echo "[dexproj] warning: failed to list container recordings for sync." >&2
            return
        fi

        mapfile -t new_episodes < <(comm -13 "$HOST_EPISODE_SNAPSHOT" "$CONTAINER_EPISODE_SNAPSHOT")
        if [ "${#new_episodes[@]}" -eq 0 ]; then
            echo "[dexproj] no new recordings to sync back."
            return
        fi

        local rel_path
        local copied=0
        for rel_path in "${new_episodes[@]}"; do
            mkdir -p "$ROOT_DIR/data/raw/$(dirname "$rel_path")"
            if docker cp "$CONTAINER_NAME:$CONTAINER_WORKDIR/data/raw/$rel_path" "$ROOT_DIR/data/raw/$(dirname "$rel_path")/" >/dev/null 2>&1; then
                copied=$((copied + 1))
                echo "[dexproj] synced recording: $rel_path"
            else
                echo "[dexproj] warning: failed to sync recording: $rel_path" >&2
            fi
        done

        if [ "$copied" -gt 0 ]; then
            echo "[dexproj] synced $copied new recording(s) back to host."
        fi
    }

    snapshot_host_episodes
    set +e
    "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/run_session.sh" "$@"
    status=$?
    set -e
    sync_recordings_back
    exit "$status"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"

python3 -m dexproj.session "$@"
