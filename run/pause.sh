#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
echo "暂停任务并尝试站立，不是急停；断线或动作忙时请使用独立急停。"
exec ros2 service call /automation/enable std_srvs/srv/SetBool '{data: false}'
