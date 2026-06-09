#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
CONTAINER_WORKDIR="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
SYNC_MARKER_REL="data/.sync_episode_request"

sync_episode_watcher() {
    local session_pid="$1"
    while kill -0 "$session_pid" 2>/dev/null; do
        if docker exec "$CONTAINER_NAME" test -f "$CONTAINER_WORKDIR/$SYNC_MARKER_REL" 2>/dev/null; then
            rel_episode="$(docker exec "$CONTAINER_NAME" cat "$CONTAINER_WORKDIR/$SYNC_MARKER_REL" | tr -d '\r\n')"
            if [ -n "$rel_episode" ]; then
                mkdir -p "$ROOT_DIR/data/$(dirname "$rel_episode")"
                docker cp "$CONTAINER_NAME:$CONTAINER_WORKDIR/data/$rel_episode" "$ROOT_DIR/data/$rel_episode"
                docker exec "$CONTAINER_NAME" rm -f "$CONTAINER_WORKDIR/$SYNC_MARKER_REL"
                echo "[dexproj] synced episode to $ROOT_DIR/data/$rel_episode"
            else
                docker exec "$CONTAINER_NAME" rm -f "$CONTAINER_WORKDIR/$SYNC_MARKER_REL" 2>/dev/null || true
            fi
        fi
        sleep 0.5
    done
}

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    HOST_EPISODE_SNAPSHOT="$(mktemp)"
    CONTAINER_EPISODE_SNAPSHOT="$(mktemp)"

    cleanup_sync_state() {
        rm -f "$HOST_EPISODE_SNAPSHOT" "$CONTAINER_EPISODE_SNAPSHOT"
    }

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

    sync_logs_back() {
        mkdir -p "$ROOT_DIR/data/logs"
        if docker cp "$CONTAINER_NAME:$CONTAINER_WORKDIR/data/logs/." "$ROOT_DIR/data/logs/" >/dev/null 2>&1; then
            echo "[dexproj] synced session logs to $ROOT_DIR/data/logs/"
        else
            echo "[dexproj] warning: failed to sync session logs from container." >&2
        fi
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
    sync_episode_watcher "$$" &
    SYNC_PID=$!
    cleanup_sync_watcher() {
        kill "$SYNC_PID" 2>/dev/null || true
        wait "$SYNC_PID" 2>/dev/null || true
        cleanup_sync_state
    }
    trap cleanup_sync_watcher EXIT INT TERM
    set +e
    "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/run_session.sh" "$@"
    status=$?
    set -e
    sync_recordings_back
    sync_logs_back
    exit "$status"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"

python3 -m dexproj.session "$@"
