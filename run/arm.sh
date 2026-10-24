#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
echo "确认遥控器初始化、踩点、路线配置和急停准备均完成。输入 RUN 才会允许运动："
read -r answer
[[ "$answer" == RUN ]] || exit 1
exec ros2 service call /automation/enable std_srvs/srv/SetBool '{data: true}'
