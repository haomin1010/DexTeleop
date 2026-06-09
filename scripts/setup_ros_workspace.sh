#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
CONTAINER_WORKDIR="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
ROS_WS="${DEXPROJ_ROS_WS:-/home/wuji/ros2_ws}"
USE_CHINA_MIRRORS="${DEXPROJ_USE_CHINA_MIRRORS:-1}"
PIP_INDEX_URL="${DEXPROJ_PIP_INDEX_URL:-https://pypi.org/simple}"

if [ "$USE_CHINA_MIRRORS" = "1" ]; then
    PIP_INDEX_URL="${DEXPROJ_PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
fi

if ! command -v docker >/dev/null 2>&1; then
    echo "[dexproj] docker not found on host. Please install Docker first." >&2
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo "[dexproj] cannot access Docker. Add your user to the docker group or use a shell with Docker permissions." >&2
    echo "[dexproj] suggested fix: sudo usermod -aG docker \$USER && newgrp docker" >&2
    exit 1
fi

if ! docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "[dexproj] container '$CONTAINER_NAME' not found." >&2
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    echo "[dexproj] starting existing container '$CONTAINER_NAME'..." >&2
    docker start "$CONTAINER_NAME" >/dev/null
fi

echo "[dexproj] syncing DexProj workspace into container '$CONTAINER_NAME'..."
docker exec "$CONTAINER_NAME" mkdir -p "$(dirname "$CONTAINER_WORKDIR")"
docker exec "$CONTAINER_NAME" rm -rf "$CONTAINER_WORKDIR"
docker cp "$ROOT_DIR/." "$CONTAINER_NAME:$CONTAINER_WORKDIR"

if ! docker exec "$CONTAINER_NAME" bash -lc 'command -v colcon >/dev/null 2>&1'; then
    echo "[dexproj] colcon not found, installing colcon extensions..."
    docker exec -u root \
        -e DEXPROJ_USE_CHINA_MIRRORS="$USE_CHINA_MIRRORS" \
        "$CONTAINER_NAME" bash -lc '
set -euo pipefail
if [ "${DEXPROJ_USE_CHINA_MIRRORS:-1}" = "1" ]; then
    if [ ! -f /etc/apt/sources.list.d/dexproj-original-sources.tar ]; then
        tar -cf /etc/apt/sources.list.d/dexproj-original-sources.tar \
            /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true
    fi
    sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g" \
        -e "s|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g" \
        /etc/apt/sources.list
    find /etc/apt/sources.list.d -type f \( -name "*.list" -o -name "*.sources" \) -print0 | \
        xargs -0 -r sed -i \
            -e "s|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g" \
            -e "s|http://archive.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g" \
            -e "s|http://security.ubuntu.com/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ubuntu|g"
fi
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y python3-colcon-common-extensions
'
fi

echo "[dexproj] building ROS2 workspace in '$ROS_WS'..."
docker exec \
    -e DEXPROJ_PIP_INDEX_URL="$PIP_INDEX_URL" \
    "$CONTAINER_NAME" bash -lc "
set -euo pipefail
mkdir -p '$ROS_WS/src'
ln -sfn '$CONTAINER_WORKDIR/wuji-hand-teleop/src' '$ROS_WS/src/wuji-hand-teleop-src'
source '$CONTAINER_WORKDIR/scripts/activate_dexproj_env.sh'
python3 - <<'PY' || python3 -m pip install --timeout 120 --retries 8 -i \"\${DEXPROJ_PIP_INDEX_URL}\" 'empy==3.3.4' catkin_pkg lark
import em
import catkin_pkg.package
import lark
PY
cd '$ROS_WS'
filter_existing_prefixes() {
    local var_name="\$1"
    local raw_value="\${!var_name:-}"
    local filtered=""
    local path

    IFS=':' read -ra paths <<< "\$raw_value"
    for path in "\${paths[@]}"; do
        if [ -n "\$path" ] && [ -e "\$path" ]; then
            if [ -z "\$filtered" ]; then
                filtered="\$path"
            else
                filtered="\$filtered:\$path"
            fi
        fi
    done
    export "\$var_name=\$filtered"
}
filter_existing_prefixes AMENT_PREFIX_PATH
filter_existing_prefixes CMAKE_PREFIX_PATH
filter_existing_prefixes COLCON_PREFIX_PATH
rm -rf \
    build/wujihand_msgs install/wujihand_msgs \
    build/wujihand_driver install/wujihand_driver \
    build/common_input install/common_input \
    build/wuji_glove_input_py install/wuji_glove_input_py \
    build/wujihand_output install/wujihand_output \
    build/tianji_output install/tianji_output \
    build/controller install/controller \
    build/wuji_teleop_bringup install/wuji_teleop_bringup
colcon build --symlink-install --packages-select \
    common_input \
    openvr_input \
    wuji_glove_input_py \
    wujihand_output \
    tianji_output \
    controller \
    wuji_teleop_bringup \
    --cmake-args -DPYTHON_EXECUTABLE:FILEPATH=\"\$(command -v python3)\"
CONDA_PYTHON=\"\$(command -v python3)\"
while IFS= read -r script_file; do
    if head -n 1 \"\$script_file\" | grep -q '^#!/usr/bin/python3'; then
        sed -i \"1c#!\$CONDA_PYTHON\" \"\$script_file\"
    fi
done < <(find install -path '*/lib/*/*' -type f)
set +u
source install/setup.bash
set -u
ros2 pkg prefix wuji_teleop_bringup
"

echo "[dexproj] ROS2 workspace prepared."
