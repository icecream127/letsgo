#!/usr/bin/env bash
RUN_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$RUN_DIR/.." && pwd)"
LEGACY_WS="$PROJECT_ROOT/lidar_ws"
if [[ ! -f "$LEGACY_WS/install/local_setup.bash" ]]; then
    echo "请先运行 bash $RUN_DIR/build_ros2.sh" >&2
    return 1
fi
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
source "$LEGACY_WS/install/local_setup.bash"
export LD_LIBRARY_PATH="$PROJECT_ROOT/.local/lib:${LD_LIBRARY_PATH:-}"
export LEGACY_ZONE_CONFIG="$LEGACY_WS/src/ros2_to_stm32/config/map_zones_config_2.json"
for package in fast_lio livox_ros_driver2 ros2_to_stm32; do
    prefix="$(ros2 pkg prefix "$package")" || return 1
    case "$prefix" in "$LEGACY_WS"/install/*) ;; *) echo "错误：$package 来自其他工作空间：$prefix" >&2; return 1 ;; esac
done
cd "$PROJECT_ROOT"
