#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

is_hand_only() {
    hand_only=0
    hand_only_docker="${DEXPROJ_HAND_ONLY_DOCKER:-0}"
    dry_run=0
    for arg in "$@"; do
        case "$arg" in
            --hand-only)
                hand_only=1
                ;;
            --docker)
                hand_only_docker=1
                ;;
            --dry-run)
                dry_run=1
                ;;
        esac
    done
}

resolve_hand_teleop_config() {
    hand_teleop_config="config/hand_teleop_wuji_glove.yaml"
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --hand-teleop-config)
                hand_teleop_config="$2"
                shift 2
                ;;
            --hand-teleop-config=*)
                hand_teleop_config="${1#*=}"
                shift
                ;;
            *)
                shift
                ;;
        esac
    done
}

resolve_hand_backend() {
    hand_backend="${DEXPROJ_HAND_ONLY_BACKEND:-}"
    while [ "$#" -gt 0 ]; do
        case "$1" in
            --hand-backend)
                hand_backend="$2"
                shift 2
                ;;
            --hand-backend=*)
                hand_backend="${1#*=}"
                shift
                ;;
            *)
                shift
                ;;
        esac
    done

    if [ -z "$hand_backend" ] && [ -f "$hand_teleop_config" ]; then
        hand_backend="$(python3 - "$hand_teleop_config" <<'PY'
from pathlib import Path
import sys

try:
    import yaml
except ImportError:
    print("")
    raise SystemExit(0)

path = Path(sys.argv[1])
try:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
except Exception:
    data = {}
print(str(data.get("backend", "") or ""))
PY
)"
    fi

    case "${hand_backend:-ros2}" in
        py)
            hand_backend="teleop_real"
            ;;
        ros|ros2)
            hand_backend="ros2"
            ;;
        teleop_real)
            hand_backend="teleop_real"
            ;;
        *)
            echo "[dexproj] unsupported hand backend: $hand_backend (expected ros2 or py)" >&2
            exit 2
            ;;
    esac
}

auto_update_glove_sn_config() {
    if [ "$dry_run" -eq 1 ]; then
        return
    fi
    if [ ! -f "$hand_teleop_config" ]; then
        return
    fi
    local auto_discover
    auto_discover="$(python3 - "$hand_teleop_config" <<'PY'
from pathlib import Path
import sys

try:
    import yaml
except ImportError:
    print("false")
    raise SystemExit(0)

path = Path(sys.argv[1])
try:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
except Exception:
    data = {}
print("true" if data.get("auto_discover_glove_sn", False) else "false")
PY
)"
    if [ "$auto_discover" = "true" ]; then
        python3 -m dexproj.tools.wuji_glove_sn --update-config "$hand_teleop_config"
    fi
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
            --hand-backend)
                shift 2
                ;;
            --hand-backend=*)
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
resolve_hand_teleop_config "$@"
resolve_hand_backend "$@"

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    if [ "$hand_only" -eq 1 ] && [ "$hand_backend" = "teleop_real" ]; then
        if [ "$hand_only_docker" != "1" ]; then
            run_teleop_real "$@"
        fi
    fi
fi

if [ -z "${DEXPROJ_RUNNING_IN_CONTAINER:-}" ] && [ ! -f "/.dockerenv" ]; then
    exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" "scripts/bringup_teleop.sh" "$@"
fi

if [ "$hand_only" -eq 1 ] && [ "$hand_backend" = "teleop_real" ]; then
    activate_container_conda
    auto_update_glove_sn_config
    run_teleop_real "$@"
fi

source "$ROOT_DIR/scripts/activate_dexproj_env.sh"
auto_update_glove_sn_config

bringup_args=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --hand-backend)
            shift 2
            ;;
        --hand-backend=*)
            shift
            ;;
        *)
            bringup_args+=("$1")
            shift
            ;;
    esac
done

python3 -m dexproj.integration.bringup "${bringup_args[@]}"
