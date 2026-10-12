# 交接：2026-10-12 上机体检 + D13/D14 修复

日期：2026-10-12
本次提交：
- 上午 `2ccb67b` — 修复级联文件路径与 abot_find 包名误报，新增上机操作清单与体检记录
- 下午 `6ec7281` — 上机验证 C5/C7 并建成本房间地图：修 killall 漏杀、悬空软链接误报，导航改用 my_home
上一份交接：[`handoff_2026-10-10_github-init.md`](handoff_2026-10-10_github-init.md)
问题总账：[`docs/上机调试问题汇总.md`](docs/上机调试问题汇总.md)
上机操作单：[`docs/上机操作清单.md`](docs/上机操作清单.md)

---

## 一、本次做了什么

**A. U 盘 ↔ 仓库同步比对（第 0 步）**

11 个上次改过的文件逐字节比对，全部一致；`abot/src` 全树递归比对无差异；
`maps/` 一致；`abot/` 顶层文件清单完全相同。U 盘 `src/` 下三个空壳
（`build`/`install`/`log`）确认已清除。

**B. 顺带清理**

仓库工作树里遗留的 `abot/src/robot_slam/maps/my_lab.yaml.bak2`
（内容是修 C2 **之前**的旧坏路径 `/home/abot/abot/src/robot_slam/maps/my_lab.pgm`）
已移到 `D:\robot-car-backup\robot-car-stale-backups\`。
它被 `.gitignore` 排除、不会入库，但物理上躺在 `maps/` 里，
属于「从仓库拷回 U 盘就会把坏路径带回去」的那类隐患 —— 与 A2 空壳同一个教训。**✅ 已搬走**

**C. 上机跑 `0-check.sh` 体检（第 1 步）✅ 通过**

车上处于**干净停用状态**，逐节结论：

- ✅ **A2 未复发** —— `ROS_PACKAGE_PATH=/media/abot/999/abot/src:/opt/ros/noetic/share`，
  `rospack find robot_slam` 正确解析到 `src/robot_slam`；另外 5 个关键包
  （`abot_bringup` / `cam_track` / `lidar_filters` / `yolov8_ros`）全部指向 `src/<包名>`
- ✅ **C2 未复发** —— `my_lab.yaml` 的 `image:` 指向
  `/media/abot/999/abot/src/robot_slam/maps/my_lab.pgm`，且 pgm 真实存在
- ✅ 4 个地图文件齐全（`my_lab.pgm` 541748 B、`my_room.pgm` 918581 B）
- ✅ 串口别名正确（`abot_driver -> ttyUSB1`、`rplidar -> ttyUSB0`）⇒ **USB 插口没换**（E1 守住）
- ✅ `ttyUSB0/1/2` 三个都在；`video0~3` 四个都在
- ✅ **正在占用串口的进程：（无）**；**前台残留 ROS 进程：（无）**；roscore 没跑
- ⬜ 第 1/7 节（`src/` 空壳的显式检查）与第 5/7 节（模型资源清单）**尚未采集**
- ✅ `/home/abot/yolov8` 存在 ⇒ `multi_yolo.py`、`cv_counter.py` 的硬编码路径**继续保持原样**

**D. 修掉两个问题**

- ✅ **D13** —— `opencv_demo/launch/face_detection.launch:18,21` 的人脸/眼睛级联文件
  原先写死 `/home/abot/cv2/opencv-4.5.5/data/haarcascades/*.xml`（源码目录，本机不存在）。
  全盘搜确认真实位置是系统自带的 **`/usr/share/opencv4/haarcascades/`**，两行已改。
- ✅ **D14**（新发现）—— `abot_find` **只是目录名**，`package.xml` 里声明的包名是
  `find_object_2d`，包内 launch 也全用 `pkg="find_object_2d"`。
  所以 `rospack find abot_find` 失败是正常的 —— 这解释了 `0-check.sh` 第 6 节那条
  自相矛盾的 `[!!] abot_find 找不到`。已改 `0-check.sh` 按包名查并显式提示；
  `功能测试清单.md` 里的命令改成 `roslaunch find_object_2d find_object_2d.launch`。

**E. 新增文档**

- ✅ `docs/上机操作清单.md` —— **带到车跟前照着敲**的操作单。每步给出
  命令 / 期望结果 / 出问题怎么办；附常用速查、还原点表、进度表。
- ✅ `docs/上机调试问题汇总.md` 新增「2026-10-12 上机体检」一节，逐节记录体检结果。

---

## 二、当前状态

| 项 | 状态 |
|---|---|
| 仓库 `main` | `2ccb67b`，工作树干净，**领先 origin 1 个提交、未推送** |
| U 盘 `E:\abot` | 上一轮已同步：`face_detection.launch`、四个启动脚本、`ros-env.sh`。<br>⚠️ **本轮有待同步的三项**（2026-10-12 傍晚插回 Windows 时处理）：<br>① `abot/killall.sh` —— 盘上**还是旧版**（缺第 3 层 `$WS/devel/lib/`+`$WS/src/`、缺 `fuser` 寻址、缺悬空软链接检查）<br>② `abot/0-check.sh` —— 第 3 节的**悬空软链接修复**（先测 `-e` 再测 `-L`）未同步<br>③ 删除 `abot/src/abot_bringup/launch/.goutputstream-RIW5J2`（仓库里已 `git rm`，盘上还在） |
| 机器人 | 体检通过，**干净起点**；`0-check.sh` 是只读的，没在车上写坏任何东西 |
| 车内 `/etc`、`/home/abot` | 一个字未动 |

> ⚠️ 注意：`face_detection.launch` 和 `0-check.sh` 的修改**已在 U 盘上**，
> 所以下次插到车上就是最新版，可以直接测 `roslaunch opencv_demo face_detection.launch`。

---

## 三、下一步（按顺序）

**第 2 步 · 验证重写后的 `killall.sh`（D10 / C3）⏸ 结论作废，须重做**

```bash
cd /media/abot/999/abot
./1-gmapping.sh            # 等 15~20 秒
./killall.sh               # 新版三层 pkill
pgrep -a -f rplidar        # 必须无输出
pgrep -a -f roslaunch      # 必须无输出
```

当时两条 `pgrep` 均为空，一度记为"通过"。**但随后查出 C5**：
环境里预置的 `ROS_HOSTNAME=172.20.10.2` / `ROS_MASTER_URI=http://172.20.10.2:11311`
是**旧手机热点**的网段，而机器真实 IP 已经变成 `10.128.202.88`，
`roscore` 连不上自己、启动即退出（`RLException: Unable to contact my own server at
[http://172.20.10.2:36113]`），`rosnode list` 无限卡住。

⇒ **那一刻 `pgrep` 为空，很可能只是"根本没东西起来"**，而不是"杀干净了"。
**D10 尚未真正验证。修好 C5 后必须重走一遍第 2 步。**
（`killall.sh` 自身打印的复查表是真实有效的，但同样不能证明被杀的东西真的启动过。）

**✅ 立即绕法（不必等 U 盘）**：在交互式终端里先导出再启动，环境会被
`gnome-terminal` 及全部标签页继承：

```bash
export ROS_HOSTNAME=localhost
export ROS_MASTER_URI=http://localhost:11311
./3-amcl_test.sh
```

**✅ 等 U 盘回 Windows 时要落的三处改动 —— 已于 2026-10-12 全部落完**：

1. **✅ 已落** —— 四个启动脚本（`1-gmapping.sh` / `3-amcl_test.sh` / `4-test.sh` /
   `grasp.sh`）都在 `gnome-terminal` 那行**之前**加了 `export ROS_HOSTNAME=localhost` +
   `export ROS_MASTER_URI=http://localhost:11311`；另新增 `abot/ros-env.sh`，
   供**手工**起节点时 `source`。已同步到 U 盘，全树比对零差异。
2. **✅ 已修** —— `abot/1-gmapping.sh` **第 6 行**行尾多一个空格：`...exec bash"' \ ` ——
   shell 里 `\ ` 是"转义空格"而**不是续行符**，续行失效、`gnome-terminal` 多收一个
   空格参数。它是最后一行、本来不需要续行，故危害有限，但属于"可能悄悄吞掉最后一个
   标签页"的那类脏东西。**修法**：删掉那个空格。（`3-amcl_test.sh` / `4-test.sh` /
   `grasp.sh` 收尾均正常。）
3. 把 C5 记进 `docs/上机调试问题汇总.md` —— **✅ 本次已在仓库侧写好**（含根因、
   排除项 `/etc/hosts` 干净、修法、待查项），只差在 U 盘上落脚本改动。

---

## 三之二、2026-10-12 下午追加（C5 上机验证 + D10 揭穿）

| 项 | 结果 |
|---|---|
| **C5 上机验证** | ✅ **通过** —— `source ros-env.sh` 后 `roscore &` + `rosnode list` 打出 `/rosout`。<br>旁证：修复前 `rosnode list` **无限卡住**，修复后变成**秒失败**（`ERROR: Unable to communicate with master!`）—— 说明确实在连本机了。<br>（脚本内置的 `export` 尚未实跑，下次 `./1-gmapping.sh` 时看第一个标签页即可） |
| **D10 的"修复"被揭穿是半拉子** | `./killall.sh` 报 `[!!] abot_driver 还有 1 个残留进程`。根因：第二层只 `pkill -f "/opt/ros/noetic/lib"`，**工作区自己编译的节点不在那个路径下**（`abot_driver` 在 `devel/lib/abot_bringup/`）；源码跑的 python 节点（`.../src/...py`）同样漏网。<br>**✅ 已重写** `killall.sh`：新增第 3 层覆盖 `$WS/devel/lib/` + `$WS/src/`（`$WS` 由脚本自身位置推导），补杀还在 `sleep` 的标签页包装进程，加 `fuser -k` 串口兜底，复查表扩容（`pub_point`/`multi_yolo`/`abot_bringup`）并新增串口占用检查 |
| **新增** `abot/ros-env.sh` | 手工起节点时 `source` 用（★ 普通终端里 `ROS_MASTER_URI` 仍是死 IP，脚本的 export 救不了手敲的命令） |
| **移除** `.goutputstream-RIW5J2` | `abot/src/abot_bringup/launch/` 下厂商遗留的 gedit 临时文件（2024-02-29 随初始提交 `e75ffdb` 进库）。已备份到 `D:\robot-car-backup\robot-car-stale-backups\`，git 历史里永久可查 |
| **8 个 `.sh` 的执行位** | git 索引里原本都是 `100644` ⇒ 改为 `100755`（只动索引，工作区文件未变）。exFAT 上没有权限位所以现在无感，但**从仓库克隆出的工作区会 `Permission denied`** |
| **C6 结案（原"待查"）** | 没有 roslaunch 在跑，`abot_driver` 却活着占着 `/dev/ttyUSB1`。已查实它是**某个 gnome-terminal 标签页里的前台进程**（`USER=abot`、`STAT=Ssl+`），**非 root、非 systemd、非开机自启**，`kill -9` 可杀。原先"卡在 D 状态"的猜测**已否定**。<br>**✅ 15:23 结案**：它不是在"占着串口"，是在**读一个已经死掉的串口** —— `select()` 报可读 + `read()` 返回 0（EOF）正是 `serial_transport.cpp:119` 那行 `read err %d` 的触发条件（"没数据"走超时分支、不打日志）。**C6 是 C7 的症状，不是独立问题。** |
| **新发现 C7（下午，硬件/驱动层）** | `/rplidarNode` 起不来 + `abot_driver` 刷 `read err 0`，共同根因是 **`/dev/ttyUSB*` 不存在**，`/dev/abot_driver -> ttyUSB1`、`/dev/rplidar -> ttyUSB0` 成了**悬空软链接**（14:15 建的）。<br>**⚠️ 我先前"适配器没枚举出来"的说法不准确**：15:23 `lsusb` 显示三个串口芯片**都在总线上**（`0403:6001` FTDI / `10c4:ea60` CP210x / `1a86:7523` CH340），与早上体检时的 `ttyUSB0/1/2` 三个正好对应 ⇒ **硬件在，是驱动/设备节点这层没生成 tty**。头号嫌疑是 Ubuntu 22.04 的 **brltty 抢占串口芯片**（它的 udev 规则恰好覆盖这三家，症状就是"lsusb 看得见、/dev/ttyUSB 没有"）。<br>**✅ 15:31 复查定性：设备全回来了** —— `ttyUSB0/1/2` 三个都是**真实字符设备**（`crw-rw---- root dialout 188,0/1/2`），`/sys/bus/usb-serial/devices/` 三个都在。`ttyUSB1` 的节点时间戳是 **15:29**（另两个 15:12 / 14:39）⇒ **底盘适配器中途掉过、刚重新枚举回来**，属**瞬时 USB 掉线**，不是硬件坏。<br>**brltty 确实装着但这次没抢**（真抢了 ttyUSB 根本不会出现）⇒ 降级为间歇嫌疑，**先不动**。<br>**⚠️ 设备回来 ≠ 进程自愈**：老 `abot_driver` 攥着**已 hangup 的旧 fd**，新枚举的是新节点，`read err 0` 会一直刷 —— **必须杀掉重启**。<br>另：`dmesg` 非 root 读不了，查掉线历史要 `sudo dmesg`。<br>详见 `docs/上机调试问题汇总.md` C7。 |
| **顺手加固** | ① `0-check.sh` 第 3 节、`killall.sh` 的"串口/设备节点"节：**先测 `-e` 再测 `-L`** —— 悬空软链接会骗过一切只看 `ls` 的检查（旧版就打印了 `[OK]`）。<br>② `killall.sh` 加了 `fuser` 的显式寻址（它在 `/usr/sbin`，普通用户 PATH 里没有）。<br>③ `killall.sh` 的 `fuser -k` 只对**真实存在**的设备节点动刀（悬空链接会让 fuser 报错）。 |

> 本次上机的两条无害告警（不必处理）：`Package name "TTS_audio" does not follow
> the naming conventions`（命名规范提醒）、`the rosdep view is empty`（rosdep 没初始化，
> **不要**去 `rosdep init`）。另有 gnome-terminal 的 `选项"-e"已弃用` —— 目前仍可用，
> 标签页确实开出来并执行了命令，**不是**故障原因。

## 三之三、2026-10-12 傍晚追加（C4 建图成功 + C7/C8 收口）

| 项 | 结果 |
|---|---|
| **C4 建图 ✅ 完成** | 16:21 干净会话跑 `./1-gmapping.sh` + teleop 走一圈，`map_saver -f /media/abot/999/abot/src/robot_slam/maps/my_home` 存下 **736×736 @ 0.02 m/px（14.7 m 见方）** 的本房间地图：`my_home.pgm` 541748 B / `my_home.yaml` 175 B。<br>✅ `image:` 是**绝对路径** ⇒ **C2 不复发**；✅ `md5sum` 与 `my_lab.pgm` **不同** ⇒ 是真新图、不是旧图被覆盖。<br>✅ **已肉眼确认与房间对得上**（墙直、转角成型、无拖尾）⇒ 合格可用。已同步进仓库，`md5sum` 与盘上一致（`007a3cb9…451b8`）。<br>✅ **并已把导航切到这张图**：`navigation.launch` 第 4 行 `map_name` 默认值 `my_lab.yaml` → **`my_home.yaml`**（否则三个启动脚本会继续加载厂商那张**别的房间**的图）。 |
| **C7 收口 ✅** | 定性为**瞬时 USB 掉线**（`ttyUSB1` 于 15:29 重新枚举，`ttyUSB0/1/2` 三个都是真实字符设备）。关键结论：**设备回来 ≠ 进程自愈** —— 老 `abot_driver` 攥着已 hangup 的旧 fd，`read err 0` 会一直刷，**必须杀掉重启**。重启后串口干净，未再复发。 |
| **C8 收口 ✅ 未复现** | 重启 + 干净启动后 `/robot_state_publisher` **正常在跑**，RViz 雷达扇形正常、地图正常生成；上次的 `/amcl`、`/map_server`、`/move_base` 残留全没了。又查过 `abot_bringup/` 下所有 launch，**没有任何 `required="true"`**（不存在"某节点一挂带走整包"的机制）。⇒ 判定为**脏会话偶发，不是代码 bug**，不单独修。 |
| **地图 origin 疑点（不阻塞）** | `my_home.yaml` 与 `my_lab.yaml` 的 `origin`（`[-1.0, -13.8, 0.0]`）/ `resolution` / 尺寸**逐位相同**，但内容不同（md5 不同）。查过 `gmapping.launch.xml`：**没把地图窗口写死**（`xmin/xmax/ymin/ymax=±1.0` 是扫描匹配搜索窗，不是地图尺寸）。未证实，**不影响使用**。 |
| **⭐ 第 3 步 全过（D9 关闭）** | 换 `my_home` 后跑 `./3-amcl_test.sh`：`/amcl`、`/map_server`、`/move_base`、`/rplidarNode` 四个都在；**RViz 里激光点贴合地图墙线**（定位准）；`2D Nav Goal` 的红色 `Full Plan` **绕开障碍物、不贴墙**。⇒ **D9 安全隐患正式关闭**（代码 2026-10-10 已注释掉那三行，本次是上机证实它真的生效）。 |

**第 3 步 · 导航回归 + D9 避障验证 ⭐ 最优先**

⚠️ **前置条件（C7）**：动手前先确认串口是活的，否则雷达起不来、`abot_driver` 会刷屏，
后面的结论全是废的：

```bash
ls -l /dev/ttyUSB*                 # 必须先看到 ttyUSB0/1/2
ls /sys/bus/usb-serial/devices/    # 不能是空的（空 = 驱动没绑上）
```

没有的话，按 C7 的流程走：`dmesg | grep -iE "ttyUSB|cp210x|ch341|brltty"` 和
`dpkg -l | grep -i brltty` 取证 → `sudo reboot` → 再验一次。

```bash
./3-amcl_test.sh
rosnode list | grep -E "amcl|map_server|move_base|rplidarNode"   # 四个都要在
```
RViz 里用 `2D Nav Goal` 给目标点，看红色 `Full Plan` **必须绕开障碍物、不贴墙**。
贴墙说明 `pub_point.py` 那三行 `enabled: 0` 还在起作用。

**之后**：第 4 步查相机（B1）、第 5 步重新建图（C4，存图必须用绝对路径
`map_saver -f /media/abot/999/abot/src/robot_slam/maps/my_home`）、第 6 步清 D 系列。

**待补**：第 1/7、5/7 节的截图（不阻塞开工）。

---

## 四、本次改动文件清单

| 文件 | 改动 |
|---|---|
| `abot/src/opencv_demo/launch/face_detection.launch` | D13，两行级联路径 |
| `abot/0-check.sh` | D14，第 6 节 `[B]` 改按包名查 |
| `docs/上机调试问题汇总.md` | 新增体检一节；D13 标已修；新增 D14 行 |
| `docs/功能测试清单.md` | 模板匹配命令改正；相机那行改为 D5 作废后的新结论 |
| `docs/上机操作清单.md` | **新增** |

**还原方式**：源码两处可 `git revert 2ccb67b`；搬走的 `my_lab.yaml.bak2` 在
`D:\robot-car-backup\robot-car-stale-backups\`，`mv` 回去即可。

### 下午那一批（提交 `6ec7281`）

| 文件 | 改动 |
|---|---|
| `abot/1-gmapping.sh` / `3-amcl_test.sh` / `4-test.sh` / `grasp.sh` | C5：`gnome-terminal` 之前加两行 `export`；顺手修掉 `1-gmapping.sh` 第 6 行行尾那个"转义空格"（`\ ` 不是续行符） |
| `abot/ros-env.sh` | **新增** —— 手敲节点时 `source` 用（脚本的 export 救不了手敲命令） |
| `abot/killall.sh` | **整脚本重写**：补第 3 层 `$WS/devel/lib/` + `$WS/src/`；`fuser` 显式寻址；悬空软链接感知 |
| `abot/0-check.sh` | 第 3 节串口检查改为**先测 `-e` 再测 `-L`**；第 4 节改为从 `navigation.launch` **读当前用图** |
| `abot/src/robot_slam/maps/my_home.{pgm,yaml}` | **新增** —— 自建本房间地图（736×736 @ 0.02 m/px） |
| `abot/src/robot_slam/launch/navigation.launch` | **导航默认图 `my_lab.yaml` → `my_home.yaml`** |
| `abot/src/abot_bringup/launch/.goutputstream-RIW5J2` | **删除** —— 厂商遗留的 gedit 临时文件（备份在 `D:\robot-car-backup\robot-car-stale-backups\`） |
| 8 个 `.sh` | 补执行位（`100644` → `100755`，只动索引，工作区文件未变） |
| `docs/上机操作清单.md` / `上机调试问题汇总.md` / `功能测试清单.md` | C7/C8 新增、C4 收口、C5 验证记录、地图说明 |

**还原方式**：`git revert 6ec7281`。若只想退回厂商地图，改
`navigation.launch` 第 4 行的 `map_name` 回 `my_lab.yaml` 即可（`my_lab` 仍留在 `maps/` 下）。
