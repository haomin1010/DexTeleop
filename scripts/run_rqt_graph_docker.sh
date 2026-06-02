#!/usr/bin/env bash
# Launch only rqt_graph inside the DexProj Docker container.
#
# Usage:
#   ./scripts/run_rqt_graph_docker.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_DEXPROJ="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"

exec "$ROOT_DIR/scripts/ensure_docker_exec.sh" -- bash -lc "
set -euo pipefail
source $CONTAINER_DEXPROJ/scripts/activate_dexproj_env.sh
cd /home/wuji/ros2_ws
set +u
source install/setup.bash
set -u
export QT_X11_NO_MITSHM=1
PIP_INDEX_URL=\"\${DEXPROJ_PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}\"
PIP_TRUSTED_HOST=\"\${DEXPROJ_PIP_TRUSTED_HOST:-pypi.tuna.tsinghua.edu.cn}\"
PIP_EXTRA_INDEX_URL=\"\${DEXPROJ_PIP_EXTRA_INDEX_URL:-}\"
MISSING_PKGS=()

python3 -c 'import PyQt5' >/dev/null 2>&1 || MISSING_PKGS+=(PyQt5)
python3 -c 'import pydot' >/dev/null 2>&1 || MISSING_PKGS+=(pydot)

if [ \"\${#MISSING_PKGS[@]}\" -gt 0 ]; then
  if [ \"\${DEXPROJ_SKIP_AUTO_PYQT_INSTALL:-0}\" = \"1\" ]; then
    echo \"[dexproj] missing Python deps for rqt_graph: \${MISSING_PKGS[*]}\" >&2
    echo '[dexproj] auto install is disabled by DEXPROJ_SKIP_AUTO_PYQT_INSTALL=1.' >&2
    exit 2
  fi
  echo \"[dexproj] installing missing deps via pip mirror: \$PIP_INDEX_URL\" >&2
  echo \"[dexproj] packages: \${MISSING_PKGS[*]}\" >&2
  PIP_ARGS=(--index-url \"\$PIP_INDEX_URL\" --trusted-host \"\$PIP_TRUSTED_HOST\")
  if [ -n \"\$PIP_EXTRA_INDEX_URL\" ]; then
    PIP_ARGS+=(--extra-index-url \"\$PIP_EXTRA_INDEX_URL\")
  fi
  python3 -m pip install --user \"\${PIP_ARGS[@]}\" \"\${MISSING_PKGS[@]}\"
fi

exec ros2 topic echo /tianji_arm/right/joint_command
"
