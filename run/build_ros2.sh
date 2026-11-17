#!/usr/bin/env bash
set -eo pipefail
if [[ "${PROJECT_CLEAN_ENV:-0}" != 1 ]]; then
    exec env -i HOME="$HOME" USER="${USER:-user}" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 PROJECT_CLEAN_ENV=1 BUILD_JOBS="${BUILD_JOBS:-2}" bash "$0" "$@"
fi
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
cp -- "$PROJECT_ROOT/lidar_ws/src/livox_ros_driver2/package_ROS2.xml" "$PROJECT_ROOT/lidar_ws/src/livox_ros_driver2/package.xml"
source /opt/ros/humble/setup.bash
cmake -S "$PROJECT_ROOT/third_party/Livox-SDK2" -B "$PROJECT_ROOT/.build_sdk" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PROJECT_ROOT/.local"
cmake --build "$PROJECT_ROOT/.build_sdk" --parallel "${BUILD_JOBS:-2}"
cmake --install "$PROJECT_ROOT/.build_sdk"
export CMAKE_PREFIX_PATH="$PROJECT_ROOT/.local:${CMAKE_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$PROJECT_ROOT/.local/lib:${LD_LIBRARY_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS:-2}"
cd "$PROJECT_ROOT/lidar_ws"
sources=(src)
packages=(ros2_to_stm32)
options=(-DBUILD_TESTING=OFF)
if [[ "${1:-}" == --gicp ]]; then
    revision=fd29d8cf94cf05ed7ad21c81c27b65963110adb5
    dependency="$PROJECT_ROOT/.local/small_gicp"
    if [[ ! -d "$dependency/.git" ]]; then
        git clone https://github.com/koide3/small_gicp.git "$dependency"
    fi
    if [[ -n "$(git -C "$dependency" status --porcelain)" ]]; then
        echo 'small_gicp dependency has local changes' >&2
        exit 1
    fi
    git -C "$dependency" checkout --detach "$revision"
    sources+=("$dependency")
    packages+=(go2_real_localization)
    options=(-DBUILD_WITH_OPENMP=ON -DBUILD_TESTING=ON)
elif [[ $# != 0 ]]; then
    echo 'Usage: bash run/build_ros2.sh [--gicp]' >&2
    exit 2
fi
colcon build --base-paths "${sources[@]}" --packages-up-to "${packages[@]}" --executor sequential --symlink-install --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble "${options[@]}" "-DLIVOX_LIDAR_SDK_LIBRARY=$PROJECT_ROOT/.local/lib/liblivox_lidar_sdk_shared.so" "-DLIVOX_LIDAR_SDK_INCLUDE_DIR=$PROJECT_ROOT/.local/include"
