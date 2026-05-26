#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

DEFAULT_UPDATE_CONFIG="config/hand_teleop_wuji_glove.yaml"

resolve_update_config() {
    update_config=""
    if [ "$#" -eq 0 ]; then
        update_config="$DEFAULT_UPDATE_CONFIG"
        return
    fi
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --update-config)
                update_config="$2"
                shift 2
                ;;
            --update-config=*)
                update_config="${1#*=}"
                shift
                ;;
            *)
                shift
                ;;
        esac
    done
}

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    resolve_update_config "$@"
    if [ -n "$update_config" ]; then
        if [ "$#" -eq 0 ]; then
            "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/get_wuji_glove_sn.sh" \
                --update-config "$DEFAULT_UPDATE_CONFIG"
        else
            "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/get_wuji_glove_sn.sh" "$@"
        fi
        container_name="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
        container_workdir="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
        if [[ "$update_config" = /* ]]; then
            container_config="$update_config"
            host_config="$update_config"
        else
            container_config="$container_workdir/$update_config"
            host_config="$ROOT_DIR/$update_config"
        fi
        docker cp "$container_name:$container_config" "$host_config" >/dev/null
        exit 0
    fi
    exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/get_wuji_glove_sn.sh" "$@"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"

if [ "$#" -eq 0 ]; then
    python3 -m dexproj.tools.wuji_glove_sn --update-config "$DEFAULT_UPDATE_CONFIG"
else
    python3 -m dexproj.tools.wuji_glove_sn "$@"
fi
