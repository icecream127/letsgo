#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/environment.sh"
case "${1:-}" in --check|--dry-run|--ordered|'') ;; *) echo "Usage: $0 [--check|--dry-run|--ordered]" >&2; exit 2 ;; esac
python3 -c 'import rclpy, serial, tkinter, matplotlib; from ros2_to_stm32 import info_transmiting, trans, ui'
if [[ "${1:-}" == --check ]]; then echo "环境检查通过，未启动节点。"; exit 0; fi
scripts=(lio_driver.sh ui.sh pos_trans.sh serial.sh info_trans.sh)
titles=(fastlio GUI pos serial info)
terminal_args=()
if [[ "${1:-}" != --dry-run ]]; then
    command -v gnome-terminal >/dev/null
    echo "启动传感器、界面和控制节点，默认不允许运动。初始化和踩点后再运行 bash run/arm.sh。"
fi
for index in "${!scripts[@]}"; do
    if [[ "${1:-}" == --dry-run ]]; then
        printf '%s: bash %s\n' "${titles[$index]}" "$SCRIPT_DIR/${scripts[$index]}"
        continue
    fi
    delay=0
    if [[ "${1:-}" == --ordered ]]; then delay=$((index * 3)); fi
    printf -v tab_command 'bash -c %q _ %q %q' 'sleep "$2"; bash "$1"; exec bash' "$SCRIPT_DIR/${scripts[$index]}" "$delay"
    if [[ "$index" == 0 ]]; then
        terminal_args+=(--window)
    else
        terminal_args+=(--tab)
    fi
    terminal_args+=(--title="${titles[$index]}" --command="$tab_command")
done
if [[ "${1:-}" != --dry-run ]]; then
    gnome-terminal "${terminal_args[@]}"
fi
