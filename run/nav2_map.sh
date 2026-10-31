#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 ]]; then echo "Usage: bash run/nav2_map.sh <map_name> [use_sim_time:=true]" >&2; exit 2; fi
map_name="$1"
shift
source "$SCRIPT_DIR/environment.sh"
map_file="$(/usr/bin/python3 "$SCRIPT_DIR/maps.py" validate "$map_name")"
exec ros2 launch ros2_to_stm32 nav2_map.launch.py "$@" "map:=$map_file"
