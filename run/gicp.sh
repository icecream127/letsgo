#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
ENTRY=bringup.launch.py
if [[ "${1:-}" == "--localizer-only" ]]; then
    ENTRY=localization.launch.py
    shift
fi
MAP_PATH="${GICP_MAP_PATH:-$PROJECT_ROOT/maps/real_room_20261121_112353_068513.pcd}"
[[ -f "$MAP_PATH" ]] || { echo "PCD 不存在：$MAP_PATH" >&2; exit 1; }
for package in fast_gicp go2_real_localization; do
    prefix="$(ros2 pkg prefix "$package")"
    [[ "$prefix" == "$LEGACY_WS"/install/* ]] || { echo "错误：$package 来自其他工作空间：$prefix" >&2; exit 1; }
done
exec ros2 launch go2_real_localization "$ENTRY" map_path:="$MAP_PATH" "$@"
