#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER_NAME="${DEXPROJ_CONTAINER_NAME:-wuji22-hand}"
CONTAINER_WORKDIR="${DEXPROJ_CONTAINER_WORKDIR:-/workspace/DexProj}"
USE_CHINA_MIRRORS="${DEXPROJ_USE_CHINA_MIRRORS:-1}"
MINICONDA_INSTALLER_NAME="Miniconda3-py310_25.3.1-1-Linux-x86_64.sh"
MINICONDA_INSTALLER_URL="${DEXPROJ_MINICONDA_URL:-https://repo.anaconda.com/miniconda/${MINICONDA_INSTALLER_NAME}}"
PIP_INDEX_URL="${DEXPROJ_PIP_INDEX_URL:-https://pypi.org/simple}"
CONDA_ROOT_CANDIDATES=(
    "/opt/miniconda3"
    "/home/wuji/miniconda3"
)

if [ "$USE_CHINA_MIRRORS" = "1" ]; then
    MINICONDA_INSTALLER_URL="${DEXPROJ_MINICONDA_URL:-https://mirrors.tuna.tsinghua.edu.cn/anaconda/miniconda/${MINICONDA_INSTALLER_NAME}}"
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

docker start "$CONTAINER_NAME" >/dev/null || true

echo "[dexproj] preparing runtime inside container '$CONTAINER_NAME'..."

docker exec -i \
    -e DEXPROJ_USE_CHINA_MIRRORS="$USE_CHINA_MIRRORS" \
    -e DEXPROJ_MINICONDA_URL="$MINICONDA_INSTALLER_URL" \
    -e DEXPROJ_PIP_INDEX_URL="$PIP_INDEX_URL" \
    "$CONTAINER_NAME" bash <<'EOS'
set -euo pipefail

if [ "${DEXPROJ_USE_CHINA_MIRRORS:-1}" = "1" ]; then
    cat > "$HOME/.condarc" <<'CONDARC'
channels:
  - defaults
show_channel_urls: true
default_channels:
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/main
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/r
  - https://mirrors.tuna.tsinghua.edu.cn/anaconda/pkgs/msys2
CONDARC
fi

CONDA_ROOT=""
for candidate in /opt/miniconda3 /home/wuji/miniconda3; do
    if [ -f "$candidate/etc/profile.d/conda.sh" ]; then
        CONDA_ROOT="$candidate"
        break
    fi
done

if [ -z "$CONDA_ROOT" ]; then
    CONDA_ROOT="/home/wuji/miniconda3"
    echo "[dexproj] no conda installation found, installing Miniconda to $CONDA_ROOT ..."
    TMP_INSTALLER="/tmp/miniconda.sh"
    curl -fL --retry 5 --retry-delay 2 "${DEXPROJ_MINICONDA_URL}" -o "$TMP_INSTALLER"
    bash "$TMP_INSTALLER" -b -p "$CONDA_ROOT"
    rm -f "$TMP_INSTALLER"
fi

source "$CONDA_ROOT/etc/profile.d/conda.sh"

if ! conda env list | awk '{print $1}' | grep -Fxq dexproj; then
    conda create -y -n dexproj python=3.10
fi

conda run -n dexproj pip install --no-cache-dir \
    --timeout 120 --retries 8 \
    -i "${DEXPROJ_PIP_INDEX_URL}" \
    pyyaml inputs pytest numpy \
    wuji-sdk==0.10.0 wujihandpy avp_stream scipy nlopt pin
EOS

echo
echo "[dexproj] container runtime prepared."
echo "[dexproj] Important:"
echo "  1. make sure DexProj is available inside the container at $CONTAINER_WORKDIR"
echo "  2. make sure the ROS2 workspace has rebuilt wuji_glove_input_py"
echo "  3. then run ./scripts/start_docker.sh"
