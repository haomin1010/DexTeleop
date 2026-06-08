#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS_ROOT="${DEXPROJ_ROS_WS_ROOT:-$HOME/ros2_ws}"
ROS_WS_SRC="$ROS_WS_ROOT/src"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
CONDA_SH="${DEXPROJ_CONDA_SH:-$HOME/miniconda3/etc/profile.d/conda.sh}"
CONDA_ENV_NAME="${DEXPROJ_CONDA_ENV:-dexproj}"
BUILD_CAMERA="${DEXPROJ_BUILD_CAMERA:-1}"
ROSDEP_SKIP_KEYS="${DEXPROJ_ROSDEP_SKIP_KEYS:-ament_python wuji_retargeting}"

source_compat() {
    local had_u=0
    case $- in
        *u*) had_u=1 ;;
    esac

    set +u
    # shellcheck disable=SC1090
    source "$1"
    if [ "$had_u" -eq 1 ]; then
        set -u
    fi
}

if [ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
    echo "[dexproj] ROS 2 ${ROS_DISTRO_NAME} not found at /opt/ros/${ROS_DISTRO_NAME}." >&2
    exit 1
fi

mkdir -p "$ROS_WS_SRC"
ln -sfn "$ROOT_DIR/wuji-hand-teleop/src" "$ROS_WS_SRC/wuji-hand-teleop-src"

source_compat "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"

if [ -f "$CONDA_SH" ]; then
    source_compat "$CONDA_SH"
    conda activate "$CONDA_ENV_NAME"
fi

cd "$ROS_WS_ROOT"

echo "[dexproj] resolving rosdep dependencies for $ROS_WS_ROOT ..."
rosdep install --from-paths src --ignore-src -r -y --skip-keys "$ROSDEP_SKIP_KEYS"

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
    tianji_urdf
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

CONDA_PYTHON="$(command -v python3)"
while IFS= read -r script_file; do
    if head -n 1 "$script_file" | grep -q '^#!/usr/bin/python3'; then
        sed -i "1c#!$CONDA_PYTHON" "$script_file"
    fi
done < <(find install -path '*/lib/*/*' -type f)

echo
echo "[dexproj] native ROS workspace prepared."
echo "[dexproj] source with:"
echo "  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
echo "  source \"$ROS_WS_ROOT/install/setup.bash\""
