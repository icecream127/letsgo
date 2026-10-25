#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
echo "单向发送：无法确认电控状态。确认遥控器已初始化并按23开启自动、A/B踩点及起跑位置正确、路线和独立急停准备完成。遥控器接管后必须先暂停上位机，禁止直接再次按23续跑。输入 RUN 才会允许运动："
read -r answer
[[ "$answer" == RUN ]] || exit 1
exec ros2 service call /automation/enable std_srvs/srv/SetBool '{data: true}'
