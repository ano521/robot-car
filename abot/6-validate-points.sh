#!/bin/bash
# 只做已测导航点验收：不启动机械臂，不修改避障/代价地图参数。
# 运行前：./0-check.sh、./killall.sh、./3-amcl_test.sh，并在 RViz 完成 2D Pose Estimate。

set -u

WS=/media/abot/999/abot

if [ ! -f "$WS/ros-env.sh" ]; then
  echo "[!!] 找不到工作区：$WS"
  exit 1
fi

# shellcheck disable=SC1091
source "$WS/ros-env.sh"
# shellcheck disable=SC1091
source "$WS/devel/setup.bash"

exec python3 "$WS/src/robot_slam/scripts/validate_points.py" "$@"
