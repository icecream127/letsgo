#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 1 ]]; then echo "Usage: bash run/navigation.sh <map_name> [rviz:=false] [sensors:=false]" >&2; exit 2; fi
map_name="$1"
shift
source "$SCRIPT_DIR/environment.sh"
map_file="$(/usr/bin/python3 "$SCRIPT_DIR/maps.py" validate "$map_name")"
exec ros2 launch ros2_to_stm32 navigation.launch.py "$@" "map:=$map_file"
