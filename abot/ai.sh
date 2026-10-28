#!/bin/bash
# ============================================================
#  ai.sh —— `roslaunch yolov8_ros ai_grasp.launch ...` 的短版
#
#  为什么要有它：
#    包名 yolov8_ros 手打极容易漏一个 v（已经漏过一次），
#    而且新终端还得先 source 两个文件。这个脚本把三件事都包了。
#
#  用法（在任意目录都行，脚本会自己 cd）：
#     ./ai.sh              # 默认 /dev/video2，不开 GUI，不干跑
#     ./ai.sh v0           # 用 /dev/video0（机械臂摄像头）
#     ./ai.sh gui          # 开 find_object_2d 的 GUI，用来截图做模板
#     ./ai.sh v0 gui       # 两个一起
#     ./ai.sh v0 dry       # 干跑：只算不动臂
#     ./ai.sh v0 id1       # 只认 1 号模板（模板编号从 1 开始）
#
#  注意：这个脚本自己 source 环境，所以新终端里直接跑就行，
#        不用再敲 source /media/abot/999/abot/ros-env.sh
# ============================================================

WS="$(cd "$(dirname "$0")" && pwd)"
cd "$WS" || exit 1

# ---- 环境（C5：环境里预置了旧热点的死 IP，必须覆盖）----
if [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    . /opt/ros/noetic/setup.bash
fi
# shellcheck disable=SC1091
[ -f "$WS/ros-env.sh" ] && . "$WS/ros-env.sh"
# shellcheck disable=SC1091
[ -f "$WS/devel/setup.bash" ] && . "$WS/devel/setup.bash"

VIDEO=/dev/video2
ARGS=()

for a in "$@"; do
    case "$a" in
        -h|--help|help)
            sed -n '2,25p' "$0"
            exit 0
            ;;
        v0|video0)  VIDEO=/dev/video0 ;;
        v2|video2)  VIDEO=/dev/video2 ;;
        gui)        ARGS+=("gui:=true") ;;
        dry)        ARGS+=("dry_run:=true") ;;
        id*)        ARGS+=("target_id:=${a#id}") ;;
        *)          ARGS+=("$a") ;;
    esac
done

echo "工作区: $WS"
echo "相机  : $VIDEO"
echo "参数  : ${ARGS[*]:-（默认）}"
echo

exec roslaunch yolov8_ros ai_grasp.launch "video:=$VIDEO" "${ARGS[@]}"
