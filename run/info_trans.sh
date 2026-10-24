#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
exec ros2 run ros2_to_stm32 info_trans --ros-args --params-file "$PROJECT_ROOT/lidar_ws/src/ros2_to_stm32/config/control.yaml" "$@"

