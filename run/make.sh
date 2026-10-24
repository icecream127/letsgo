#!/usr/bin/env bash
set -eo pipefail
if [[ -z "${CH341_SOURCE_DIR:-}" || ! -f "$CH341_SOURCE_DIR/Makefile" ]]; then
    echo "旧 CH341 驱动源码未随包提供。如确有需要，请显式设置 CH341_SOURCE_DIR。" >&2
    exit 1
fi
cd "$CH341_SOURCE_DIR"
make
sudo make load
