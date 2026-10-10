#!/bin/bash
# ============================================================
#  ABOT 机器人 —— 只读体检脚本
#
#  只查看，不改动任何东西。跑完把屏幕截图发我。
#
#  用法：
#     cd /media/abot/999/abot
#     ./0-check.sh
# ============================================================

WS=/media/abot/999/abot

ok()   { echo "  [OK]   $1"; }
no()   { echo "  [!!]   $1"; }
warn() { echo "  [~~]   $1"; }
hr()   { echo; echo "------------------------------------------------------------"; }

hr; echo " 1/7  工作区"; hr
[ -d "$WS" ] && ok "$WS 存在" || no "$WS 不存在！U盘没插？"
for d in src build devel; do
  [ -d "$WS/$d" ] && ok "$WS/$d" || no "$WS/$d 缺失"
done
echo
echo "  厂商残留目录（应该都已清掉）:"
echo "  注：只查 src/ 下面那几个（那是当初误在 src/ 里跑 catkin_make 产生的空壳，"
echo "      rospack 会爬进去、导致 \$(find pkg) 指错）。顶层 build/ devel/ 是本工作区"
echo "      正常的编译产物，不在检查范围内。"
for d in src/build src/install src/log; do
  if [ -e "$WS/$d" ]; then
    warn "$WS/$d 还在 --> 会干扰包搜索"
  else
    ok "$WS/$d 已清除"
  fi
done

hr; echo " 2/7  ROS 包解析（本次修复的重点）"; hr
cd "$WS" 2>/dev/null
# shellcheck disable=SC1091
. /opt/ros/noetic/setup.bash 2>/dev/null
# shellcheck disable=SC1091
. "$WS/devel/setup.bash" 2>/dev/null

echo "  ROS_PACKAGE_PATH = $ROS_PACKAGE_PATH"
echo
RS=$(rospack find robot_slam 2>/dev/null)
echo "  rospack find robot_slam  ->  $RS"
case "$RS" in
  /media/abot/999/abot/src/robot_slam) ok "解析正确！" ;;
  *"/install/"*|*"/build/"*)           no "又指到空壳了，问题没解决" ;;
  "")                                  no "找不到 robot_slam 包" ;;
  *)                                   warn "解析到了非预期路径" ;;
esac
echo
echo "  其他关键包:"
for p in abot_bringup robot_slam cam_track lidar_filters yolov8_ros; do
  r=$(rospack find "$p" 2>/dev/null)
  [ -n "$r" ] && echo "    $p  ->  $r" || no "$p 找不到"
done

hr; echo " 3/7  串口与相机设备"; hr
for d in /dev/abot_driver /dev/rplidar; do
  if [ -L "$d" ]; then
    ok "$d  ->  $(readlink -f "$d" 2>/dev/null)"
  elif [ -e "$d" ]; then
    ok "$d 存在"
  else
    no "$d 不存在"
  fi
done
echo
echo "  串口设备列表:"
ls -l /dev/ttyUSB* 2>/dev/null || no "没有 /dev/ttyUSB*"
echo
echo "  相机设备列表:"
ls -l /dev/video* 2>/dev/null || no "没有 /dev/video* （相机没插？）"
echo
echo "  正在占用串口的进程:"
pgrep -a -f "rplidar|abot_driver" 2>/dev/null || echo "    （无）"

hr; echo " 4/7  地图文件"; hr
for f in "$WS/src/robot_slam/maps/my_lab.pgm" "$WS/src/robot_slam/maps/my_lab.yaml" \
         "$WS/maps/my_room.pgm" "$WS/maps/my_room.yaml"; do
  if [ -s "$f" ]; then
    ok "$(basename "$f")  $(stat -c%s "$f") 字节"
  else
    no "$f 缺失或 0 字节"
  fi
done
echo
echo "  my_lab.yaml 的内容:"
sed 's/^/    /' "$WS/src/robot_slam/maps/my_lab.yaml" 2>/dev/null
IMG=$(grep '^image:' "$WS/src/robot_slam/maps/my_lab.yaml" 2>/dev/null | sed 's/^image:[[:space:]]*//')
echo
echo "  yaml 里 image 指向: $IMG"
if [ -n "$IMG" ]; then
  [ -f "$IMG" ] && ok "这个 pgm 真实存在（map_server 应该能起来了）" \
                || no "这个 pgm 不存在！map_server 还是会崩"
fi

hr; echo " 5/7  模型与资源文件"; hr
while read -r f; do
  if [ -s "$f" ]; then
    ok "$(basename "$f")  $(du -h "$f" 2>/dev/null | cut -f1)"
  else
    no "$(echo "$f" | sed "s#$WS/##") 缺失"
  fi
done <<EOF
$WS/src/robot_audio/paraformer-zh/model.pt
$WS/src/whisper/whisper-tiny-zh-ct2-int8/model.bin
$WS/src/sam/model/seg.pt
$WS/src/track/model/track.pt
$WS/src/region_counter/model/counter.pt
$WS/src/ocr_detect/ChineseFonts/simsun.ttc
$WS/src/vl_locate/config/SimHei.ttf
$WS/src/TTS_audio/speaker/speaker_5_girl.pth
EOF

hr; echo " 6/7  车内路径（代码里写死、我改不了的）"; hr
echo "  下面这些如果显示 [!!]，说明对应功能会失败："
echo
for p in /home/abot/abot /home/abot/abot_ws /home/abot/yolov8 /home/abot/cv2 \
         /home/abot/vision_ws /home/abot/qinghua; do
  if [ -e "$p" ]; then
    ok "$p 存在"
  else
    no "$p 不存在"
  fi
done
echo
echo "  /home/abot 下实际有什么:"
ls -1 /home/abot 2>/dev/null | sed 's/^/    /' || no "连 /home/abot 都读不了"

echo
echo "  ---- 待定位的替代路径（找到哪个我就用它替换硬编码）----"
echo "  [A] opencv_demo/launch/face_detection.launch 表情级联文件:"
for p in \
  /usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml \
  /usr/share/opencv/haarcascades/haarcascade_frontalface_alt.xml \
  /usr/local/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml \
  /home/abot/cv2/opencv-4.5.5/data/haarcascades/haarcascade_frontalface_alt.xml ; do
  [ -f "$p" ] && ok "$p" || no "$p"
done
echo "    全盘搜（最多 20 秒）:"
timeout 20 find / -name "haarcascade_frontalface_alt.xml" -not -path "/proc/*" 2>/dev/null \
  | sed 's/^/      /' | head -5 || true
echo "    python 的 cv2 装在哪:"
python3 -c "import cv2,os;print('      cv2',cv2.__version__,'->',os.path.dirname(cv2.__file__))" 2>/dev/null \
  || no "      python3 -c 'import cv2' 失败"
echo
echo "  [B] find_object_2d 包（abot_find 的 launch 里依赖它）:"
r=$(rospack find find_object_2d 2>/dev/null); [ -n "$r" ] && ok "$r" || no "系统里没有 find_object_2d 包"
r=$(rospack find abot_find 2>/dev/null);      [ -n "$r" ] && ok "$r" || no "abot_find 找不到"

hr; echo " 7/7  当前 ROS 运行状态"; hr
if pgrep -x rosmaster >/dev/null 2>&1; then
  ok "roscore 正在运行"
  echo
  echo "  节点列表:"
  rosnode list 2>/dev/null | sed 's/^/    /'
else
  warn "roscore 没在跑（还没启动的话这是正常的）"
fi
echo
echo "  前台残留的 ROS 进程:"
ps aux 2>/dev/null | grep -E "roslaunch|rosrun|rplidar|abot_driver|map_server|move_base" \
  | grep -v grep | awk '{print "    " $11, $12, $13}' || echo "    （无）"

hr; echo " 体检结束"; hr
