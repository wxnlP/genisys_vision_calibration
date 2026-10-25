#!/usr/bin/env bash
# 构建 protocols/ros2/genisys_msgs（msg + srv）到 protocols/ros2/install。
set -eo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$HERE/ros2"
ROS_PREFIX="${ROS2_PREFIX:-/opt/ros/${ROS_DISTRO:-humble}}"
# rosidl_adapter 用系统 Python
BUILD_PYTHON="${BUILD_PYTHON:-/usr/bin/python3}"

if [ ! -x "$BUILD_PYTHON" ]; then
  echo "未找到构建用 Python：$BUILD_PYTHON" >&2
  exit 1
fi

if [ ! -f "$ROS_PREFIX/setup.bash" ]; then
  echo "未找到 ROS2：$ROS_PREFIX/setup.bash" >&2
  echo "请设置 ROS2_PREFIX（或 ROS_DISTRO）后重试。" >&2
  exit 1
fi

source "$ROS_PREFIX/setup.bash"

if ! command -v colcon >/dev/null 2>&1; then
  echo "未找到 colcon。" >&2
  exit 1
fi

cd "$WORKSPACE"

colcon build \
  --packages-select genisys_msgs \
  --merge-install \
  --install-base install \
  --build-base build \
  --cmake-clean-cache \
  --cmake-args \
    -DPython3_EXECUTABLE="$BUILD_PYTHON" \
    -DPYTHON_EXECUTABLE="$BUILD_PYTHON"

echo
echo "已构建：$WORKSPACE/install"
echo "source $WORKSPACE/install/local_setup.bash"
