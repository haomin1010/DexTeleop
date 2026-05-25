#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

is_hand_only() {
    hand_only=0
    hand_only_docker="${DEXPROJ_HAND_ONLY_DOCKER:-0}"
    for arg in "$@"; do
        case "$arg" in
            --hand-only)
                hand_only=1
                ;;
            --docker)
                hand_only_docker=1
                ;;
        esac
    done
}

run_teleop_real() {
    runner_args=()
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --hand-only|--docker)
                shift
                ;;
            --hand-teleop-config)
                runner_args+=("$1" "$2")
                shift 2
                ;;
            --hand-teleop-config=*)
                runner_args+=("$1")
                shift
                ;;
            --conda-env)
                runner_args+=("$1" "$2")
                shift 2
                ;;
            --conda-env=*)
                runner_args+=("$1")
                shift
                ;;
            --hand)
                runner_args+=("$1" "$2")
                shift 2
                ;;
            --hand=*)
                runner_args+=("$1")
                shift
                ;;
            --startup-delay)
                runner_args+=("$1" "$2")
                shift 2
                ;;
            --startup-delay=*)
                runner_args+=("$1")
                shift
                ;;
            --dry-run)
                runner_args+=("$1")
                shift
                ;;
            *)
                shift
                ;;
        esac
    done
    exec python3 -m dexproj.tools.run_wuji_teleop_real "${runner_args[@]}"
}

activate_container_conda() {
    local conda_sh=""
    for candidate in \
        /home/wuji/miniconda3/etc/profile.d/conda.sh \
        /opt/miniconda3/etc/profile.d/conda.sh
    do
        if [ -f "$candidate" ]; then
            conda_sh="$candidate"
            break
        fi
    done
    if [ -z "$conda_sh" ]; then
        echo "[dexproj] cannot find conda.sh inside container." >&2
        exit 1
    fi
    # shellcheck disable=SC1090
    source "$conda_sh"
    conda activate "${DEXPROJ_RUNNER_CONDA_ENV:-dexproj}"
}

is_hand_only "$@"

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    if [ "$hand_only" -eq 1 ] && [ "${DEXPROJ_HAND_ONLY_BACKEND:-teleop_real}" = "teleop_real" ]; then
        if [ "$hand_only_docker" != "1" ]; then
            run_teleop_real "$@"
        fi
    fi
fi

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/bringup_teleop.sh" "$@"
fi

if [ "$hand_only" -eq 1 ] && [ "${DEXPROJ_HAND_ONLY_BACKEND:-teleop_real}" = "teleop_real" ]; then
    activate_container_conda
    run_teleop_real "$@"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"

python3 -m dexproj.integration.bringup "$@"
