#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

source "${PROJECT_ROOT}/scripts/activate_dexproj_env.sh"

MERGE_PYTHON="${MERGE_PYTHON:-python3}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${PROJECT_ROOT}/data/local_merged_lerobot_dataset}"
CHUNKS_SIZE="${CHUNKS_SIZE:-1000}"

exec "${MERGE_PYTHON}" "${PROJECT_ROOT}/local_direct_pipeline/merge_lerobot_datasets.py" \
  --output-root "${OUTPUT_ROOT}" \
  --chunks-size "${CHUNKS_SIZE}" \
  --force \
  "$@"
