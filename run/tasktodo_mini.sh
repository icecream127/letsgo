#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
# 从源码运行新增模块，无需修改 setup.py 或重新编译。
export PYTHONPATH="$PROJECT_ROOT/lidar_ws/src/ros2_to_stm32:${PYTHONPATH:-}"
export LEGACY_ZONE_CONFIG="$PROJECT_ROOT/lidar_ws/src/ros2_to_stm32/ros2_to_stm32/map_mini.json"
echo "MINI：请确保原 info_trans 控制器已关闭；复用现有定位和串口节点。"
exec python3 -m ros2_to_stm32.tasktodo_mini --ros-args --params-file "$PROJECT_ROOT/lidar_ws/src/ros2_to_stm32/config/control.yaml" "$@"
