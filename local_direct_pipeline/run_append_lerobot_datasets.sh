#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

source "${PROJECT_ROOT}/scripts/activate_dexproj_env.sh"

APPEND_PYTHON="${APPEND_PYTHON:-python3}"

exec "${APPEND_PYTHON}" "${PROJECT_ROOT}/local_direct_pipeline/append_lerobot_datasets.py" "$@"
