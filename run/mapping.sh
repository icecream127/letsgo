#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 ]]; then echo "Usage: bash run/mapping.sh <map_name> [show_rviz:=false]" >&2; exit 2; fi
map_name="$1"
shift
source "$SCRIPT_DIR/environment.sh"
map_dir="$(/usr/bin/python3 "$SCRIPT_DIR/maps.py" prepare "$map_name")"
echo "地图目标目录：$map_dir；采集后另开终端执行 bash run/save_map.sh $map_name"
exec ros2 launch ros2_to_stm32 real_lidar.launch.py "$@" save_pcd:=true "map_file_path:=$map_dir/cloud.pcd"
