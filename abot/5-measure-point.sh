#!/bin/bash
# ============================================================
#  量点位 —— 把机器人"现在在哪、朝哪"读成 grab.launch 要的三个数
#
#  背景：nav.py（grasp.sh / 4-test.sh 跑的路线脚本）按 grab.launch 里
#        写死的 16 个槽位走一圈。厂商那 16 个数是他们自己实验室的，
#        要在本房间自跑就得换成自己量的。
#
#  ★★ 本房间只有 10 个位置，槽位有 15 个 ★★
#      nav.py 实际用到槽位 0~14（第 15 个从头到尾没引用）。
#      10 个位置 → 15 个槽位 的对应关系写死在下面的 SLOT_SRC 表里。
#      所以本脚本只要你量 10 个位置，--emit 自动铺满 15 个槽位。
#
#  本房间的 10 个位置（2026-10-15 实测）：
#      每组三个点 = 识别点 → 抓取位置 → 放置台，共 3 组，外加 1 个终点
#      0/3/6 = 识别点（离货架远的那一站）   2/5/8 = 抓取位置（第 N 次 /grab）
#      1/4/7 = 放置台（同一张，跑了三次各量一次）   9 = 终点
#
#  用法：
#     ./5-measure-point.sh                 # 打印位置/槽位对照表 + 当前进度
#     ./5-measure-point.sh 2               # 量位置 2
#     ./5-measure-point.sh 2 抓取位1        # 同上，记个名字
#     ./5-measure-point.sh --list          # 看已经量了哪些、还差哪些
#     ./5-measure-point.sh --emit          # 生成三行 param（没量齐会拒绝）
#
#  前置：./3-amcl_test.sh 正在跑（要有 /amcl_pose），且 RViz 里已给过
#        2D Pose Estimate —— AMCL 没收敛时读出来的坐标是垃圾，脚本会警告。
#
#  详细角色表 / 坑列表见 docs/点位测量表.md
# ============================================================

WS="$(cd "$(dirname "$0")" && pwd)"
CSV="$WS/points_measured.csv"

# ---- ROS 环境 ----
# 环境里预置了【旧手机热点】的 ROS_MASTER_URI（见 ros-env.sh / 问题汇总 C5），
# 不覆盖的话每一条 ROS 命令都会连到一个不存在的地址、无限卡住。
if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi
if [ -f "$WS/ros-env.sh" ]; then
  source "$WS/ros-env.sh"
fi

# 输出里有中文；万一落到非 UTF-8 的 locale（LC_ALL=C 之类）python 会直接抛
# UnicodeEncodeError，一行字都打不出来。写死它，省得现场猜。
export PYTHONIOENCODING=utf-8

# ---- 要实际去量的 10 个位置，编号 0~9 ────────────────────────────
declare -a ROLE=(
  "识别点 1   · 货架 1，先看清再靠近"          # 0
  "放置台     · 第 1 次 /put"                  # 1
  "抓取位置 1 · 第 1 次 /grab"                 # 2
  "识别点 2   · 货架 2"                        # 3
  "放置台     · 第 2 次 /put（同一张台子）"     # 4
  "抓取位置 2 · 第 2 次 /grab"                 # 5
  "识别点 3   · 货架 3"                        # 6
  "放置台     · 第 3 次 /put（同一张台子）"     # 7
  "抓取位置 3 · 第 3 次 /grab"                 # 8
  "终点       · 跑完停这儿"                    # 9
)

# ---- 槽位 → 位置（核心映射表）────────────────────────────────
#   nav.py 按 0,1,2,...,14 的顺序 goto，SLOT_SRC[槽位] 就是它该去哪个位置。
#   -1 = 不用（nav.py 不引用它）。
#
#   nav.py 的实际顺序（[方括号]是触发机械臂动作的地方）：
#     12 → 0[grab] → 14 → 1[put] → 2 → 3 → 13 → 4[grab] → 5[put]
#        → 6 → 7 → 8[grab] → 9[put] → 10 → 11
#
#   要改路线就改这一行。改完用 --list 复看一眼对不对。
declare -a SLOT_SRC=(
#  槽位: 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
          2  1  3  3  5  4  6  6  8  7  9  9  0  5  1 -1
)

declare -a REQUIRED=(0 1 2 3 4 5 6 7 8 9)
REQUIRED_STR="$(IFS=,; echo "${REQUIRED[*]}")"
SLOT_SRC_STR="$(IFS=,; echo "${SLOT_SRC[*]}")"

usage() {
  echo "用法："
  echo "    ./5-measure-point.sh              # 本说明 + 对照表 + 进度"
  echo "    ./5-measure-point.sh <位置编号> [名字]   # 量一个位置"
  echo "    ./5-measure-point.sh --list       # 已量的 / 还差的 / 槽位怎么铺"
  echo "    ./5-measure-point.sh --emit       # 生成三行 param"
  echo
  echo "★ 本房间 10 个位置（编号 0~9）—— 只量这些："
  echo
  for i in "${REQUIRED[@]}"; do
    printf "  %2d  %s\n" "$i" "${ROLE[$i]}"
  done
  echo
  echo "★ 铺进 nav.py 的 15 个槽位（脚本自动铺，不用管）："
  echo
  echo "   槽位: 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14"
  printf "   位置:"
  for i in $(seq 0 14); do printf "%3d" "${SLOT_SRC[$i]}"; done
  echo
  echo "   （15 号槽位 nav.py 不引用，不用管）"
  echo
  if [ -f "$CSV" ]; then
    n=$(grep -vc '^#' "$CSV" 2>/dev/null)
    echo "当前进度：已量 $n / 10（明细见 ./5-measure-point.sh --list）"
  else
    echo "当前进度：还没量过（$CSV 还不存在）"
  fi
  echo
  echo "怎么用："
  echo "  1) 用 3-amcl_test.sh 起导航（只有它带键盘遥控那一页），"
  echo "     RViz 里先给一次 2D Pose Estimate。"
  echo "  2) 把车遥控到目标位置、朝向摆正、【停稳】，然后："
  echo "         ./5-measure-point.sh 2 抓取位1"
  echo "  3) 10 个量完：  ./5-measure-point.sh --emit"
  echo "     把打出来的三行 param 覆盖进 src/robot_slam/launch/grab.launch"
}

# ============================================================
#  --list
# ============================================================
if [ "$1" = "--list" ]; then
  python3 - "$CSV" "$REQUIRED_STR" "$SLOT_SRC_STR" "${ROLE[@]}" <<'PY'
import sys, os
csv_path, req_s, slot_s, roles = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4:14]
required = [int(v) for v in req_s.split(',')]
slot_src = [int(v) for v in slot_s.split(',')]

data = {}
if os.path.exists(csv_path):
    for line in open(csv_path, encoding='utf-8'):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        f = line.split(',')
        if len(f) >= 5:
            data[int(f[0])] = f[1:]

print("要量的 10 个位置：")
for i in required:
    if i in data:
        name, x, y, yaw = data[i]
        print("  [OK] %2d  %-26s x=%8.3f  y=%8.3f  yaw=%8.2f  (%s)"
              % (i, roles[i], float(x), float(y), float(yaw), name or '-'))
    else:
        print("  [  ] %2d  %s" % (i, roles[i]))

print()
print("铺进 nav.py 的槽位（自动，不用管）：")
print("   槽位: " + "".join("%5d" % i for i in range(15)))
print("   位置: " + "".join("%5d" % slot_src[i] for i in range(15)))
ok = all(slot_src[i] in data for i in range(15))
print("   " + ("[OK] 15 个槽位都有数据了" if ok else "[--] 还有槽位没数据（等位置量齐）"))

missing = [i for i in required if i not in data]
print()
print("还差：%s" % (", ".join(str(i) for i in missing) if missing else "无 —— 可以 --emit 了"))
PY
  exit 0
fi

# ============================================================
#  --emit ：生成 grab.launch 的三行 param
# ============================================================
if [ "$1" = "--emit" ]; then
  python3 - "$CSV" "$WS" "$REQUIRED_STR" "$SLOT_SRC_STR" "${ROLE[@]}" <<'PY'
import sys, os, re

csv_path, ws, req_s, slot_s, roles = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5:15]
required = [int(v) for v in req_s.split(',')]
slot_src = [int(v) for v in slot_s.split(',')]

data = {}
if os.path.exists(csv_path):
    for line in open(csv_path, encoding='utf-8'):
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        f = line.split(',')
        if len(f) >= 5:
            data[int(f[0])] = [float(f[2]), float(f[3]), float(f[4])]

missing = [i for i in required if i not in data]
if missing:
    print("[!!] 还没量齐，拒绝生成 —— 缺这 %d 个：" % len(missing))
    for i in missing:
        print("       %2d  %s" % (i, roles[i]))
    sys.exit(1)

slots = {}
for i in range(16):
    if i < len(slot_src) and slot_src[i] >= 0 and slot_src[i] in data:
        slots[i] = data[slot_src[i]]

# ---- 顺手体检 ----
warn = []
try:
    nav = os.path.join(ws, 'src/robot_slam/launch/navigation.launch')
    txt = open(nav, encoding='utf-8').read()
    name = re.search(r'name="map_name"\s+default="([^"]+)"', txt).group(1)
    ypath = os.path.join(ws, 'src/robot_slam/maps', name)
    ytxt = open(ypath, encoding='utf-8').read()
    ox, oy = [float(v) for v in re.search(r'origin:\s*\[\s*([-\d.]+),\s*([-\d.]+)', ytxt).groups()]
    res = float(re.search(r'resolution:\s*([-\d.]+)', ytxt).group(1))
    # pgm 尺寸：P5 头是 "P5 / 注释行 / 宽 高 / 最大值" —— 只需前 200 字节
    pgm = os.path.join(ws, 'src/robot_slam/maps',
                       os.path.basename(re.search(r'image:\s*(\S+)', ytxt).group(1)))
    with open(pgm, 'rb') as f:
        head = f.read(200).decode('ascii', 'ignore').split('\n')
    dims = [l for l in head if l.strip() and not l.startswith('#')]
    w, h = (int(v) for v in dims[1].split()[:2])
    x0, x1, y0, y1 = ox, ox + w * res, oy, oy + h * res
    print("[--] 当前地图 %s：x ∈ [%.2f, %.2f]，y ∈ [%.2f, %.2f]" % (name, x0, x1, y0, y1))
    for i in required:
        x, y = data[i][0], data[i][1]
        if not (x0 <= x <= x1 and y0 <= y <= y1):
            warn.append("位置 %d 落在地图外（x=%.2f y=%.2f）—— 多半是当时 AMCL 没收敛" % (i, x, y))
except Exception as e:
    print("[--] 跳过地图范围体检：%s" % e)

# 只揪「几乎重合」的：本房间设计上 识别点↔抓取位置 本来就差 0.2 m，
# 那是有意的，不该报。真正要怀疑的是差不到 10 cm 的。
for a_i in range(len(required)):
    for b_i in range(a_i + 1, len(required)):
        i, j = required[a_i], required[b_i]
        a, b = data[i], data[j]
        d = ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
        if d < 0.10:
            warn.append("位置 %d 和 %d 只差 %.3f m —— 是同一处（放置台跑三次）吗？"
                        "是就没问题；不是就要重测" % (i, j, d))

if warn:
    print()
    for w_ in warn:
        print("[--] %s" % w_)

print()
print("把下面三行覆盖进 src/robot_slam/launch/grab.launch（整块替换）：")
print()
order = [i for i in range(16) if i in slots]
print('      <param name="goalListX" value="%s"/>'   % ",".join("%7.2f" % slots[i][0] for i in order))
print('      <param name="goalListY" value="%s"/>'   % ",".join("%7.2f" % slots[i][1] for i in order))
print('      <param name="goalListYaw" value="%s"/>' % ",".join("%7.1f" % slots[i][2] for i in order))
print()
print("[--] 共 %d 个数/行（10 个位置铺满 15 个槽位%s）"
      % (len(order), "，第 15 位未用" if 15 not in slots else ""))
print("     这些数来自 points_measured.csv；对照表见脚本顶部的 SLOT_SRC")
PY
  exit $?
fi

# ============================================================
#  量一个位置
# ============================================================
if [ -z "$1" ]; then
  usage
  exit 0
fi

if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
  usage
  exit 0
fi

IDX="$1"
NAME="${2:-}"

case "$IDX" in
  ''|*[!0-9]*)
    echo "[!!] 位置编号必须是 0~9 的数字，收到的是：$IDX"
    echo
    usage
    exit 1
    ;;
esac
if [ "$IDX" -gt 9 ]; then
  echo "[!!] 本房间要量的位置编号是 0~9，没有 $IDX。"
  echo
  echo "     你敲的 $IDX 大概是【槽位】号 —— 槽位和位置不是一回事："
  for s in $(seq 0 15); do
    src="${SLOT_SRC[$s]}"
    if [ "$s" -eq "$IDX" ]; then
      if [ "$src" -lt 0 ]; then
        echo "       槽位 $s → nav.py 不引用它，不用管"
      else
        echo "       槽位 $s → 用的是【位置 $src】（${ROLE[$src]}）"
      fi
    fi
  done
  exit 1
fi

if [ ! -f "$CSV" ]; then
  echo "# idx,name,x,y,yaw   —— 5-measure-point.sh 记录，量完用 --emit 生成 param" > "$CSV"
fi

python3 - "$CSV" "$IDX" "$NAME" "${ROLE[$IDX]}" "$REQUIRED_STR" <<'PY'
import sys, os, math, time

csv_path, idx, name, role, req_s = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5]
required = [int(v) for v in req_s.split(',')]
total = len(required)

try:
    import rospy
    from geometry_msgs.msg import PoseWithCovarianceStamped
except ImportError as e:
    print("[!!] 导不进 rospy（%s）" % e)
    print("     —— ROS 环境没 source 上。本脚本已尝试 /opt/ros/noetic/setup.bash")
    sys.exit(2)

def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def ang_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return d

print("位置 %s · %s" % (idx, role))
print("正在读 /amcl_pose（采样 3 次、约 1.5 秒）...")

rospy.init_node('measure_point', anonymous=True, disable_signals=True)

msgs = []
hourglass = time.time() + 15.0
try:
    while len(msgs) < 3 and time.time() < hourglass:
        msgs.append(rospy.wait_for_message(
            '/amcl_pose', PoseWithCovarianceStamped, timeout=max(1.0, hourglass - time.time())))
        time.sleep(0.6)
except Exception:
    pass

if not msgs:
    print()
    print("[!!] 15 秒内没等到 /amcl_pose —— AMCL 根本没在跑。")
    print("     先确认 ./3-amcl_test.sh 起来了：rosnode list | grep -E 'amcl|move_base'")
    print("     并确认 RViz 里已经给过【2D Pose Estimate】。")
    sys.exit(1)

xs = [m.pose.pose.position.x for m in msgs]
ys = [m.pose.pose.position.y for m in msgs]
yaws = [yaw_of(m.pose.pose.orientation) for m in msgs]
last = msgs[-1]

x, y = sum(xs) / len(xs), sum(ys) / len(ys)
yaw = yaws[0] + sum(ang_diff(v, yaws[0]) for v in yaws) / len(yaws)
yaw_deg = math.degrees(yaw)

drift_pos = max(((a - x) ** 2 + (b - y) ** 2) ** 0.5 for a, b in zip(xs, ys))
drift_yaw = max(abs(math.degrees(ang_diff(v, yaw))) for v in yaws)

cov = last.pose.covariance
sx, sy = math.sqrt(max(cov[0], 0.0)), math.sqrt(max(cov[7], 0.0))
syaw = math.degrees(math.sqrt(max(cov[35], 0.0)))

print()
print("[OK] 位置 %s · %s" % (idx, role))
print("       x   = %8.3f m" % x)
print("       y   = %8.3f m" % y)
print("       yaw = %8.2f °      （0 = 朝 +X，-90 = 朝 -Y）" % yaw_deg)
print("       抖动：位置 %.3f m / 朝向 %.2f °（3 次采样）" % (drift_pos, drift_yaw))
print("       不确定度：σx=%.3f m  σy=%.3f m  σyaw=%.2f °" % (sx, sy, syaw))

bad = False
if drift_pos > 0.03 or drift_yaw > 3.0:
    bad = True
    print("       [!!] 车还在动，或者定位在飘 —— 等车【完全停稳】再量一次。")
if sx > 0.05 or sy > 0.05:
    bad = True
    print("       [!!] AMCL 的位置不确定度偏大（σ>5cm）—— 多半没给 2D Pose Estimate，"
          "或者激光没贴合墙线。先修定位，再量。")
if syaw > 8.0:
    bad = True
    print("       [!!] 朝向不确定度偏大（σyaw>8°）—— 同上。")

print()
print("     >>> 这个位置的三个数：  %.2f, %.2f, %.1f" % (x, y, yaw_deg))

lines = []
if os.path.exists(csv_path):
    lines = [l for l in open(csv_path, encoding='utf-8')
             if l.startswith('#') or l.strip().split(',')[0] != idx]
if not lines:
    lines = ["# idx,name,x,y,yaw\n"]
with open(csv_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)
    f.write("%s,%s,%.4f,%.4f,%.2f\n" % (idx, name, x, y, yaw_deg))

have = set()
for l in open(csv_path, encoding='utf-8'):
    if not l.startswith('#') and l.strip():
        have.add(int(l.split(',')[0]))
missing = [i for i in required if i not in have]
print("     >>> 已记入 %s" % csv_path)
print("         进度 %d/%d，还差：%s" % (total - len(missing), total,
                                       ", ".join(str(i) for i in missing) if missing else "无 —— 可以 --emit 了"))

if bad:
    print()
    print("     ★ 上面有 [!!]：这个数【不建议】直接用，重测一次再记（重测会覆盖本行）。")
PY
