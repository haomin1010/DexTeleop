#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MINICONDA_ROOT="${DEXPROJ_MINICONDA_ROOT:-$HOME/miniconda3}"
CONDA_ENV_NAME="${DEXPROJ_CONDA_ENV:-dexproj}"
CONDA_PYTHON_VERSION="${DEXPROJ_CONDA_PYTHON_VERSION:-3.10}"
MINICONDA_INSTALLER_NAME="Miniconda3-py310_25.3.1-1-Linux-x86_64.sh"
MINICONDA_INSTALLER_URL="${DEXPROJ_MINICONDA_URL:-https://repo.anaconda.com/miniconda/${MINICONDA_INSTALLER_NAME}}"

if [ ! -f "$MINICONDA_ROOT/etc/profile.d/conda.sh" ]; then
    tmp_installer="$(mktemp --suffix=.sh)"
    echo "[dexproj] installing Miniconda to $MINICONDA_ROOT ..."
    curl -fL --retry 5 --retry-delay 2 "$MINICONDA_INSTALLER_URL" -o "$tmp_installer"
    bash "$tmp_installer" -b -p "$MINICONDA_ROOT"
    rm -f "$tmp_installer"
fi

# shellcheck disable=SC1090
source "$MINICONDA_ROOT/etc/profile.d/conda.sh"

if ! conda env list | awk '{print $1}' | grep -Fxq "$CONDA_ENV_NAME"; then
    echo "[dexproj] creating conda env '$CONDA_ENV_NAME' (python=$CONDA_PYTHON_VERSION) ..."
    conda create -y -n "$CONDA_ENV_NAME" "python=${CONDA_PYTHON_VERSION}"
fi

conda activate "$CONDA_ENV_NAME"

echo "[dexproj] installing Python dependencies into '$CONDA_ENV_NAME' ..."
python -m pip install --upgrade pip
python -m pip install --no-cache-dir \
    pyyaml pytest inputs numpy scipy \
    openvr==2.12.1401 \
    opencv-python==4.13.0.92 \
    wuji-sdk==0.10.0 \
    wujihandpy==1.7.0 \
    avp_stream==2.51 \
    pin==3.9.0 \
    nlopt==2.10.0 \
    empy==3.3.4 \
    catkin_pkg \
    lark

echo "[dexproj] installing local wuji-retargeting into '$CONDA_ENV_NAME' ..."
python -m pip install -e "$ROOT_DIR/wuji-retargeting"

echo
echo "[dexproj] native Python runtime prepared."
echo "[dexproj] activate with:"
echo "  source \"$MINICONDA_ROOT/etc/profile.d/conda.sh\""
echo "  conda activate $CONDA_ENV_NAME"
