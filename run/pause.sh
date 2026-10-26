#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
echo "人工暂停并请求站立（不是急停），bash run/arm.sh 恢复。最直接的停法是把遥控器 KEY_mode 拨回手动。"
exec ros2 service call /automation/enable std_srvs/srv/SetBool '{data: false}'
