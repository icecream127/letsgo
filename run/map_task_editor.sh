#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
map_name="${1:-competition}"

if [[ "$map_name" == "-h" || "$map_name" == "--help" ]]; then
    echo '用法：bash run/map_task_editor.sh [地图名]（默认 competition）'
    exit 0
fi
if (( $# > 1 )) || [[ ! "$map_name" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$ ]]; then
    echo '请提供一个有效的地图名，例如 competition 或 trial。' >&2
    exit 2
fi

map_yaml="$script_dir/../maps/$map_name/map.yaml"
if [[ ! -f "$map_yaml" ]]; then
    echo "找不到地图：maps/$map_name/map.yaml" >&2
    exit 1
fi

exec /usr/bin/python3 "$script_dir/map_task_editor.py" \
    --map "$map_yaml" \
    --serve \
    --open
