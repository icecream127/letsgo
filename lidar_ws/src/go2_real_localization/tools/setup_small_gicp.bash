#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace="${1:-$(cd "$script_dir/../../.." && pwd)}"
target="$workspace/src/small_gicp"
revision=fd29d8cf94cf05ed7ad21c81c27b65963110adb5
if [[ ! -e "$target" ]]; then
  git -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=30 clone https://github.com/koide3/small_gicp.git "$target"
  git -C "$target" checkout --detach "$revision"
elif [[ ! -d "$target/.git" ]] || [[ "$(git -C "$target" rev-parse HEAD)" != "$revision" ]]; then
  echo "Existing $target is not the pinned version; preserve it and resolve manually." >&2
  exit 1
fi
echo "small_gicp ready: $target ($revision)"
