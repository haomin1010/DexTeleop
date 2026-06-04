#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${PROJECT_ROOT}"

source "${PROJECT_ROOT}/scripts/activate_dexproj_env.sh"

BUILD_PYTHON="${BUILD_PYTHON:-python3}"
RAW_ROOT="${RAW_ROOT:-${PROJECT_ROOT}/data/raw_data}"
OUTPUT_ROOT="${OUTPUT_ROOT:-${PROJECT_ROOT}/data/local_lerobot_dataset}"
TARGET_HZ="${TARGET_HZ:-0.0}"
CHUNKS_SIZE="${CHUNKS_SIZE:-1000}"
ROBOT_TYPE="${ROBOT_TYPE:-tianji_dual_arm_with_dexterous_hand}"
DEFAULT_TASK="${DEFAULT_TASK:-}"

DO_RSYNC=0
FORWARD_ARGS=()

for arg in "$@"; do
  case "$arg" in
    --rsync)
      DO_RSYNC=1
      ;;
    *)
      FORWARD_ARGS+=("$arg")
      ;;
  esac
done

echo "[local_direct_pipeline] RAW_ROOT=${RAW_ROOT}" >&2
echo "[local_direct_pipeline] OUTPUT_ROOT=${OUTPUT_ROOT}" >&2
echo "[local_direct_pipeline] BUILD_PYTHON=${BUILD_PYTHON}" >&2
echo "[local_direct_pipeline] TARGET_HZ=${TARGET_HZ}" >&2
echo "[local_direct_pipeline] CHUNKS_SIZE=${CHUNKS_SIZE}" >&2
echo "[local_direct_pipeline] ROBOT_TYPE=${ROBOT_TYPE}" >&2
echo "[local_direct_pipeline] DEFAULT_TASK=${DEFAULT_TASK}" >&2
echo "[local_direct_pipeline] EXTRA_ARGS=${FORWARD_ARGS[*]:-<none>}" >&2

"${BUILD_PYTHON}" "${PROJECT_ROOT}/local_direct_pipeline/build_groot_lerobot_dataset.py" \
  --source-dir "${RAW_ROOT}" \
  --output-dir "${OUTPUT_ROOT}" \
  --target-hz "${TARGET_HZ}" \
  --chunks-size "${CHUNKS_SIZE}" \
  --robot-type "${ROBOT_TYPE}" \
  --default-task "${DEFAULT_TASK}" \
  --overwrite-output \
  "${FORWARD_ARGS[@]}"

if [[ "${DO_RSYNC}" -eq 1 ]]; then
  REMOTE_HOST="${REMOTE_HOST:-}"
  REMOTE_ROOT="${REMOTE_ROOT:-}"

  if [[ -z "${REMOTE_HOST}" || -z "${REMOTE_ROOT}" ]]; then
    echo "[local_direct_pipeline] REMOTE_HOST and REMOTE_ROOT are required when --rsync is used." >&2
    exit 1
  fi

  REMOTE_DST="${REMOTE_HOST}:${REMOTE_ROOT}"
  rsync -avP "${OUTPUT_ROOT}" "${REMOTE_DST}"
fi
