#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOCKER_DIR="$ROOT_DIR/wuji-hand-teleop/docker"
CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
AUTO_START_MODE="${DEXPROJ_DOCKER_AUTOSTART_MODE:-existing}"
CONTAINER_WORKDIR="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"

prepare_container_workspace() {
    local container_parent
    container_parent="$(dirname "$CONTAINER_WORKDIR")"
    echo "[dexproj] syncing DexProj workspace into container '$CONTAINER_NAME'..." >&2
    docker exec "$CONTAINER_NAME" mkdir -p "$container_parent"
    docker cp "$ROOT_DIR/." "$CONTAINER_NAME:$CONTAINER_WORKDIR"
}

sync_wuji_sdk_params() {
    local host_params_dir="${WUJI_SDK_PARAMS_DIR:-$HOME/.wuji/sdk/params}"
    local host_sdk_dir="${WUJI_SDK_DIR:-$(dirname "$host_params_dir")}"
    local host_models_dir="${WUJI_SDK_MODELS_DIR:-$host_sdk_dir/models}"
    local primary_params_dir="${DEXPROJ_CONTAINER_WUJI_PARAMS_DIR:-/home/wuji/.wuji/sdk/params}"
    local primary_sdk_dir
    primary_sdk_dir="$(dirname "$primary_params_dir")"
    local container_sdk_dirs=("$primary_sdk_dir" "/root/.wuji/sdk")

    if [ ! -d "$host_params_dir" ]; then
        return
    fi

    echo "[dexproj] syncing Wuji SDK calibration params/models into container '$CONTAINER_NAME'..." >&2
    local container_sdk_dir
    for container_sdk_dir in "${container_sdk_dirs[@]}"; do
        docker exec -u root "$CONTAINER_NAME" mkdir -p "$container_sdk_dir/params" "$container_sdk_dir/models"
        docker cp "$host_params_dir/." "$CONTAINER_NAME:$container_sdk_dir/params"
        if [ -d "$host_models_dir" ]; then
            docker cp "$host_models_dir/." "$CONTAINER_NAME:$container_sdk_dir/models"
        fi
        docker exec -u root "$CONTAINER_NAME" bash -lc \
            "find $(printf '%q' "$container_sdk_dir/params") -name '*.toml' -exec sed -i -E 's#^hand_model_path = \".*/([^/\"]+_hand\\.urdf)\"#hand_model_path = \"$(printf '%q' "$container_sdk_dir")/models/\\1\"#' {} +"
        docker exec -u root "$CONTAINER_NAME" bash -lc \
            "chown -R wuji:wuji $(printf '%q' "$container_sdk_dir") 2>/dev/null || true"
    done
}

if ! command -v docker >/dev/null 2>&1; then
    echo "[dexproj] docker not found on host. Please install Docker first." >&2
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "[dexproj] cannot access Docker. Add your user to the docker group or use a shell with Docker permissions." >&2
    echo "[dexproj] suggested fix: sudo usermod -aG docker \$USER && newgrp docker" >&2
    exit 1
fi

if docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "[dexproj] starting existing container '$CONTAINER_NAME'..." >&2
    docker start "$CONTAINER_NAME" >/dev/null || true
elif [ "$AUTO_START_MODE" = "compose" ]; then
    if [ ! -d "$DOCKER_DIR" ]; then
        echo "[dexproj] docker directory not found: $DOCKER_DIR" >&2
        exit 1
    fi
    echo "[dexproj] starting Docker services via compose..." >&2
    docker compose -f "$DOCKER_DIR/docker-compose.yml" up -d
else
    echo "[dexproj] container '$CONTAINER_NAME' not found." >&2
    echo "[dexproj] set DEXPROJ_DOCKER_AUTOSTART_MODE=compose to build/start the bundled container." >&2
    exit 1
fi

prepare_container_workspace
sync_wuji_sdk_params

echo
echo "[dexproj] Docker services are up."
echo "[dexproj] Next commands:"
echo "  ./scripts/get_wuji_glove_sn.sh"
echo "  ./scripts/check_devices.sh"
echo "  ./scripts/bringup_teleop.sh --dry-run --skip-preflight"
