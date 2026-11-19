#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
echo "GICP position input: do NOT run pos_trans.sh concurrently. Re-capture A/B in the map frame."
exec ros2 run ros2_to_stm32 gicp_pos_trans "$@"
