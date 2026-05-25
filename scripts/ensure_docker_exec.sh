#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_DIR="$ROOT_DIR/wuji-hand-teleop/docker"
CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
CONTAINER_WORKDIR="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
AUTO_START_MODE="${DEXPROJ_DOCKER_AUTOSTART_MODE:-existing}"

prepare_container_workspace() {
    local container_parent
    container_parent="$(dirname "$CONTAINER_WORKDIR")"
    echo "[dexproj] syncing DexProj workspace into container '$CONTAINER_NAME'..." >&2
    docker exec "$CONTAINER_NAME" mkdir -p "$container_parent"
    docker cp "$ROOT_DIR/." "$CONTAINER_NAME:$CONTAINER_WORKDIR"
}

sync_wuji_sdk_params() {
    local host_params_dir="${WUJI_SDK_PARAMS_DIR:-$HOME/.wuji/sdk/params}"
    local primary_params_dir="${DEXPROJ_CONTAINER_WUJI_PARAMS_DIR:-/home/wuji/.wuji/sdk/params}"
    local container_params_dirs=("$primary_params_dir" "/root/.wuji/sdk/params")

    if [ ! -d "$host_params_dir" ]; then
        return
    fi

    echo "[dexproj] syncing Wuji SDK calibration params into container '$CONTAINER_NAME'..." >&2
    local container_params_dir
    for container_params_dir in "${container_params_dirs[@]}"; do
        docker exec -u root "$CONTAINER_NAME" mkdir -p "$container_params_dir"
        docker cp "$host_params_dir/." "$CONTAINER_NAME:$container_params_dir"
        docker exec -u root "$CONTAINER_NAME" bash -lc \
            "chown -R wuji:wuji $(printf '%q' "$container_params_dir") 2>/dev/null || true"
    done
}

if [ -f "/.dockerenv" ]; then
    export DEXPROJ_RUNNING_IN_CONTAINER=1
    exit 0
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "[dexproj] docker not found on host. Please install Docker first." >&2
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "[dexproj] cannot access Docker. Add your user to the docker group or use a shell with Docker permissions." >&2
    echo "[dexproj] suggested fix: sudo usermod -aG docker \$USER && newgrp docker" >&2
    exit 1
fi

if ! docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    if [ "$AUTO_START_MODE" = "compose" ]; then
        if [ ! -d "$DOCKER_DIR" ]; then
            echo "[dexproj] docker directory not found: $DOCKER_DIR" >&2
            exit 1
        fi
        echo "[dexproj] ensuring Docker container is running via compose..." >&2
        docker compose -f "$DOCKER_DIR/docker-compose.yml" up -d >&2
    else
        echo "[dexproj] container '$CONTAINER_NAME' not found." >&2
        echo "[dexproj] set DEXPROJ_DOCKER_AUTOSTART_MODE=compose to build/start the bundled container." >&2
        exit 1
    fi
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "[dexproj] starting existing container '$CONTAINER_NAME'..." >&2
    docker start "$CONTAINER_NAME" >/dev/null
fi

prepare_container_workspace
sync_wuji_sdk_params

docker_exec_args=(-i)
if [ -t 0 ]; then
    docker_exec_args+=(-t)
fi

exec docker exec "${docker_exec_args[@]}" \
    -w "$CONTAINER_WORKDIR" \
    "$CONTAINER_NAME" \
    bash -lc "$(printf '%q ' "$CONTAINER_WORKDIR/${1}" "${@:2}")"
