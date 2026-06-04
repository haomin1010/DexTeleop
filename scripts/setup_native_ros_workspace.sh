#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS_ROOT="${DEXPROJ_ROS_WS_ROOT:-$HOME/ros2_ws}"
ROS_WS_SRC="$ROS_WS_ROOT/src"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
CONDA_SH="${DEXPROJ_CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV_NAME="${DEXPROJ_CONDA_ENV:-dexproj}"
BUILD_CAMERA="${DEXPROJ_BUILD_CAMERA:-1}"

if [ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
    echo "[dexproj] ROS 2 ${ROS_DISTRO_NAME} not found at /opt/ros/${ROS_DISTRO_NAME}." >&2
    exit 1
fi

mkdir -p "$ROS_WS_SRC"
ln -sfn "$ROOT_DIR/wuji-hand-teleop/src" "$ROS_WS_SRC/wuji-hand-teleop-src"

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [ -f "$CONDA_SH" ]; then
    # shellcheck disable=SC1090
    source "$CONDA_SH"
    conda activate "$CONDA_ENV_NAME"
fi

cd "$ROS_WS_ROOT"

echo "[dexproj] resolving rosdep dependencies for $ROS_WS_ROOT ..."
rosdep install --from-paths src --ignore-src -r -y

if ! python3 - <<'PY' >/dev/null 2>&1
import em
import catkin_pkg.package
import lark
PY
then
    echo "[dexproj] installing Python build helpers into the active environment ..."
    python3 -m pip install --no-cache-dir empy==3.3.4 catkin_pkg lark
fi

packages=(
    common_input
    openvr_input
    wuji_glove_input_py
    wujihand_output
    tianji_output
    controller
    wuji_teleop_bringup
)

if [ "$BUILD_CAMERA" = "1" ]; then
    packages+=(camera)
fi

echo "[dexproj] building ROS workspace packages: ${packages[*]}"
colcon build --symlink-install --packages-select "${packages[@]}" \
    --cmake-args -DPYTHON_EXECUTABLE:FILEPATH="$(command -v python3)"

echo
echo "[dexproj] native ROS workspace prepared."
echo "[dexproj] source with:"
echo "  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
echo "  source \"$ROS_WS_ROOT/install/setup.bash\""
