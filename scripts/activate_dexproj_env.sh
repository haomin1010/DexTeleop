#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS_SETUP="${DEXPROJ_ROS_WS_SETUP:-}"
CONDA_SH="${DEXPROJ_CONDA_SH:-}"

# ROS Humble setup scripts may read these tracing vars directly, which breaks
# under `set -u` if they are unset.
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export COLCON_TRACE="${COLCON_TRACE-}"

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

has_ros_library() {
    local library_name="$1"
    if command -v ldconfig >/dev/null 2>&1 && ldconfig -p 2>/dev/null | grep -Fq "$library_name"; then
        return 0
    fi
    find /opt/ros -name "$library_name" -print -quit 2>/dev/null | grep -q .
}

configure_rmw_implementation() {
    if [ "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" ]; then
        return
    fi
    if has_ros_library "librmw_cyclonedds_cpp.so"; then
        return
    fi

    if has_ros_library "librmw_fastrtps_cpp.so"; then
        echo "[dexproj] rmw_cyclonedds_cpp is requested but not installed; using rmw_fastrtps_cpp." >&2
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        unset CYCLONEDDS_URI
    else
        echo "[dexproj] rmw_cyclonedds_cpp is requested but not installed; letting ROS2 choose the default RMW." >&2
        unset RMW_IMPLEMENTATION
        unset CYCLONEDDS_URI
    fi
}

if [ -z "$CONDA_SH" ]; then
    for candidate in \
        /home/wuji/miniconda3/etc/profile.d/conda.sh \
        /opt/miniconda3/etc/profile.d/conda.sh
    do
        if [ -f "$candidate" ]; then
            CONDA_SH="$candidate"
            break
        fi
    done
fi

if [ -z "$ROS_WS_SETUP" ]; then
    for candidate in \
        /home/wuji/ros2_ws/install/setup.bash \
        /workspace/wuji_retargeting/install/setup.bash \
        /workspace/wuji-hand-teleop/install/setup.bash \
        /workspace/DexProj/wuji-hand-teleop/install/setup.bash \
        /home/wuji/wuji-hand-teleop/install/setup.bash
    do
        if [ -f "$candidate" ]; then
            ROS_WS_SETUP="$candidate"
            break
        fi
    done
fi

if [ -f "$CONDA_SH" ]; then
    # Preferred container path for the dedicated DexProj environment.
    # We intentionally layer conda first, then ROS 2, then the built workspace.
    source_compat "$CONDA_SH"
    conda activate dexproj
fi

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source_compat /opt/ros/humble/setup.bash
    configure_rmw_implementation
fi

if [ -f "$ROS_WS_SETUP" ]; then
    echo "[dexproj] sourcing ROS2 workspace: $ROS_WS_SETUP" >&2
    source_compat "$ROS_WS_SETUP"
    configure_rmw_implementation
else
    echo "[dexproj] no ROS2 workspace setup found; only /opt/ros/humble is active." >&2
fi

export DEXPROJ_ROOT="$ROOT_DIR"
