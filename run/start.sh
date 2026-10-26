#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
case "${1:-}" in --check|--dry-run|--ordered|'') ;; *) echo "Usage: $0 [--check|--dry-run|--ordered]" >&2; exit 2 ;; esac
python3 -c 'import rclpy, serial, tkinter, matplotlib; from ros2_to_stm32 import info_transmiting, trans, ui'
if [[ "${1:-}" == --check ]]; then echo "环境检查通过，未启动节点。"; exit 0; fi
scripts=(lio_driver.sh ui.sh pos_trans.sh serial.sh info_trans.sh)
titles=(fastlio GUI pos serial info)
if [[ "${1:-}" != --dry-run ]]; then
    command -v gnome-terminal >/dev/null
    echo "启动传感器、界面和控制节点。马起身、踩点完成后，遥控器 KEY_mode 拨到自动就开始跑路线，拨回手动就停。"
fi
for index in "${!scripts[@]}"; do
    if [[ "${1:-}" == --dry-run ]]; then
        printf '%s: bash %s\n' "${titles[$index]}" "$SCRIPT_DIR/${scripts[$index]}"
        continue
    fi
    gnome-terminal --title="${titles[$index]}" -- bash -c 'bash "$1"; exec bash' _ "$SCRIPT_DIR/${scripts[$index]}"
    if [[ "${1:-}" == --ordered ]]; then sleep 3; fi
done
