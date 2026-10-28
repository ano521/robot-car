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
#     ./ai.sh v0 dry orb   # ★ 用 Python 版匹配器（template_orb），绕开 OpenCV 4.2/4.5 冲突
#     ./ai.sh v0 orb dbg   #   再额外发 /template_orb/debug_image，可 rqt_image_view 看绿框
#
#  注意：这个脚本自己 source 环境，所以新终端里直接跑就行，
#        不用再敲 source /media/abot/999/abot/ros-env.sh
#
#  orb 什么时候用：find_object_2d 在本机必崩（段错误或 FATAL），
#  详见 交接日志-2026-10-28.md 第 8 节。重编 4.2 成功之前，用 orb 这条。
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
LAUNCH=ai_grasp.launch
ARGS=()

for a in "$@"; do
    case "$a" in
        -h|--help|help)
            sed -n '2,24p' "$0"
            exit 0
            ;;
        v0|video0)  VIDEO=/dev/video0 ;;
        v2|video2)  VIDEO=/dev/video2 ;;
        orb)        LAUNCH=ai_grasp_orb.launch ;;   # Python 匹配器，绕开 OpenCV 冲突
        dbg)        ARGS+=("dbg:=true") ;;
        gui)        ARGS+=("gui:=true") ;;
        dry)        ARGS+=("dry_run:=true") ;;
        id*)        ARGS+=("target_id:=${a#id}") ;;
        *)          ARGS+=("$a") ;;
    esac
done

echo "工作区: $WS"
echo "相机  : $VIDEO"
echo "启动  : $LAUNCH"
echo "参数  : ${ARGS[*]:-（默认）}"
echo

exec roslaunch yolov8_ros "$LAUNCH" "video:=$VIDEO" "${ARGS[@]}"
