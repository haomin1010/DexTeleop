#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
USE_CHINA_MIRRORS="${DEXPROJ_USE_CHINA_MIRRORS:-1}"

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

echo "[dexproj] installing CycloneDDS RMW in container '$CONTAINER_NAME'..."
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
DEBIAN_FRONTEND=noninteractive apt-get install -y ros-humble-rmw-cyclonedds-cpp
find /opt/ros/humble -name librmw_cyclonedds_cpp.so -print -quit
'

echo "[dexproj] CycloneDDS RMW installed."
