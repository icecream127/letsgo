#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
# 默认就是允许的；只在用 pause.sh 人工暂停过之后，用它恢复。
echo "取消人工暂停。遥控器 KEY_mode 在自动、马已起身、定位正常时就会继续跑路线。"
exec ros2 service call /automation/enable std_srvs/srv/SetBool '{data: true}'
