# 交接：2026-10-12 上机体检 + D13/D14 修复

日期：2026-10-12
本次提交：
- 上午 `2ccb67b` — 修复级联文件路径与 abot_find 包名误报，新增上机操作清单与体检记录
- 下午 `6ec7281` — 上机验证 C5/C7 并建成本房间地图：修 killall 漏杀、悬空软链接误报，导航改用 my_home
- 傍晚 `0ee50e0` — 记录第 3 步上机验证通过：AMCL 定位准、D9 避障恢复正常（安全隐患关闭）
- 傍晚 `5fc4a08` — 查清 B1 根因：RViz 的 Image 订阅的是另一台相机的话题
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
| 仓库 `main` | `5fc4a08`，工作树干净（除本交接文件外），**领先 origin 4 个提交、未推送** |
| U 盘 `E:\abot` | ✅ **2026-10-12 傍晚已全量同步**（`diff -rq` 全树零差异）。<br>⭐ **晚间第二批也已同步**（2026-10-12 17:2x，`diff -rq` 复查无差异、三个文件 md5 一致）：<br>① `abot/src/robot_slam/scripts/nav.py` —— D9 补修（注释 `enabled: 0`）<br>② `abot/src/robot_slam/scripts/vl_nav.py` —— 同上<br>③ `abot/src/yolov8_ros/launch/usb_cam.launch` —— B1 修复（加 `framerate=25`）<br>**⚠️ `docs/` 从来就不在 U 盘上**（`E:\` 根下和 `E:\abot\` 下都没有 `docs/` 目录）—— 文档是仓库专有，只落 D 盘。 |
| 机器人 | 第 3 步导航验证通过，**干净起点**；`0-check.sh` 是只读的，没在车上写坏任何东西 |
| 车内 `/etc`、`/home/abot` | 一个字未动 |

> ⚠️ 唯一**没做**的事：**没有 `git push`**。`main` 领先 `origin/main` 4 个提交，
> 全在本地。要推就说一声。（`2ccb67b` / `6ec7281` / `0ee50e0` / `5fc4a08`）

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

## 三之四、2026-10-12 晚间追加（D9 只修了半条 —— `grasp.sh` 会关避障）

**起因**：用户跑完 `./grasp.sh` 发来 RViz 截图（`屏幕截图 2026-10-12 165746.png`）。
截图本身只有两件事：`roscore http://localhost:11311/`（**C5 的 export 实锤生效**）、
`Image` 面板仍是 `No Image`（topic `/usb_cam2/image_raw`）；
另外 `Full Plan` 前面的勾**没打**（要给 `2D Nav Goal` 得先勾上）。

但顺着查 `grasp.sh` 到底起了什么，挖出一个**比截图更要紧的问题**：

| 发现 | 说明 |
|---|---|
| **🚨 `grasp.sh` 是厂商的整套演示剧本，不是"导航+抓取"** | 第 7 个标签页 `roslaunch robot_slam grab.launch` → 起 `nav.py`，里面等着 `input("Please enter 1 to continue: ")`。**这是唯一的安全闸。** |
| **🚨 D9 只修了半条** | 同样的三行 `update_configuration({"enabled": 0})`（关掉代价地图的障碍物层/静态地图层/膨胀层 ⇒ 路径穿墙）在**三个文件**里都有：`pub_point.py`（✅ 10-10 已注释）、**`nav.py`（❌ 还活着）**、**`vl_nav.py`（❌ 还活着）**。<br>`3-amcl_test.sh` 走的是 `pub_point.py` ⇒ 所以第 3 步验出"避障正常"**只证明了那一条路**；`grasp.sh` / `4-test.sh` 走的 `nav.py`，**一按 `1` 照样关避障**。<br>**✅ 已把 `nav.py` 和 `vl_nav.py` 一并注释掉**，全工作区 grep `update_configuration` + `enabled` **已无未注释调用**；三个文件均过 `ast.parse`。 |
| **🚨 `nav.py` 还有别的危害** | ① 往 `grab.launch` 里**写死的 16 个坐标**跑 —— 那是**厂商实验室**的点位<br>② **5 处 `navi.end()` 盲退**：`linear.x = -0.3` × 0.9 s ≈ **0.27 米**，全程不看传感器<br>　⚠️ **2026-10-15 更正**：原写"3 处 × 8 s ≈ 2.4 米"，把 `while time1 <= 8` 的循环次数当成了秒数 —— 实际 9 次 × `sleep(0.1)` = 0.9 s。**方向仍是盲的，结论不变**<br>③ 依次触发 `/grab`、`/put`，**动机械臂** |
| **⇒ 结论** | **`grasp.sh` ≈「在厂商房间里跑一遍厂商的剧本」。** 想让它在本房间自跑，得先量点位、换 `grab.launch` 的 `goalList`。**在那之前别按 `1`**；只想让它自己走过去，用 `2D Nav Goal`（第 3.5 步）。 |
| **⭐ B1 真因找到并已修（17:2x）** | 用户截来第 4 个标签页（`multi_yolo`），日志写死：`Starting 'head_camera' (/dev/video2) at 640x480 via mmap (yuyv) at 30 FPS` → `[ERROR] VIDIOC_STREAMON error 5, Input/output error` → `process has died`。<br>⇒ **`/usb_cam2/image_raw` 从来没人发过**，RViz 的 `No Image` 不是"话题配错"，是**那个话题不存在**。<br>**根因**：`v4l2-ctl -d /dev/video2 --list-formats-ext` 实测 —— 640×480 下 **MJPG 和 YUYV 都只到 25 fps**，而 `usb_cam` 的 `framerate` **默认 30**，`usb_cam.launch` 里又没写。请求一个不存在的帧率 ⇒ STREAMON 直接 EIO。<br>**✅ 已修**：`yolov8_ros/launch/usb_cam.launch` 显式加 `<param name="framerate" value="25" />`，**已同步到 U 盘（md5 一致）**。**尚未上机验证。** |
| **排查中排除掉的**（省得重走） | ① **不是设备被占** —— 全仓库只有 `yolov8_ros/launch/usb_cam.launch` 用 `video2`，其余全用 `video0`；`grasp.sh` 其它标签页没有一个开相机；无 `cv2.VideoCapture` 直连代码。<br>② **不是设备号错** —— `video2` 是摄像机 B 的主采集节点（`video3` 是它的元数据通道），能打开、能设成 640×480，**是到"开流"才失败**。<br>③ 那条 calibration `[WARN]` 是没做标定，**不是故障**。<br>④ 两台相机挂在**同一个 USB 控制器** `usb-3610000.xhci` 的 2.1 / 2.3 口 —— 以后再遇"开两台坏一台"先怀疑总线带宽。 |
| **修不好时的后备** | ① `pixel_format` 改 `mjpeg`（压缩，带宽小一个数量级；那行 `color_format=yuv422p` 本就是 mjpeg 才用的）<br>② MJPG 也 EIO ⇒ 设备/内核层，和 **C7 同一族** ⇒ `sudo reboot`<br>③ 重启仍不行才考虑硬件 |
| **顺带记一笔** | `cam_track` 的 `usb_cam_with_calibration.launch`（video0）**也没写 `framerate`**，同样默认 30。video0 目前没见报错，先不动；**哪天它也报 STREAMON EIO，修法一模一样**。 |

---

## 三之三、2026-10-12 傍晚追加（C4 建图成功 + C7/C8 收口）

| 项 | 结果 |
|---|---|
| **C4 建图 ✅ 完成** | 16:21 干净会话跑 `./1-gmapping.sh` + teleop 走一圈，`map_saver -f /media/abot/999/abot/src/robot_slam/maps/my_home` 存下 **736×736 @ 0.02 m/px（14.7 m 见方）** 的本房间地图：`my_home.pgm` 541748 B / `my_home.yaml` 175 B。<br>✅ `image:` 是**绝对路径** ⇒ **C2 不复发**；✅ `md5sum` 与 `my_lab.pgm` **不同** ⇒ 是真新图、不是旧图被覆盖。<br>✅ **已肉眼确认与房间对得上**（墙直、转角成型、无拖尾）⇒ 合格可用。已同步进仓库，`md5sum` 与盘上一致（`007a3cb9…451b8`）。<br>✅ **并已把导航切到这张图**：`navigation.launch` 第 4 行 `map_name` 默认值 `my_lab.yaml` → **`my_home.yaml`**（否则三个启动脚本会继续加载厂商那张**别的房间**的图）。 |
| **C7 收口 ✅** | 定性为**瞬时 USB 掉线**（`ttyUSB1` 于 15:29 重新枚举，`ttyUSB0/1/2` 三个都是真实字符设备）。关键结论：**设备回来 ≠ 进程自愈** —— 老 `abot_driver` 攥着已 hangup 的旧 fd，`read err 0` 会一直刷，**必须杀掉重启**。重启后串口干净，未再复发。 |
| **C8 收口 ✅ 未复现** | 重启 + 干净启动后 `/robot_state_publisher` **正常在跑**，RViz 雷达扇形正常、地图正常生成；上次的 `/amcl`、`/map_server`、`/move_base` 残留全没了。又查过 `abot_bringup/` 下所有 launch，**没有任何 `required="true"`**（不存在"某节点一挂带走整包"的机制）。⇒ 判定为**脏会话偶发，不是代码 bug**，不单独修。 |
| **地图 origin 疑点（不阻塞）** | `my_home.yaml` 与 `my_lab.yaml` 的 `origin`（`[-1.0, -13.8, 0.0]`）/ `resolution` / 尺寸**逐位相同**，但内容不同（md5 不同）。查过 `gmapping.launch.xml`：**没把地图窗口写死**（`xmin/xmax/ymin/ymax=±1.0` 是扫描匹配搜索窗，不是地图尺寸）。未证实，**不影响使用**。 |
| **⭐ 第 3 步 全过（D9 关闭）** | 换 `my_home` 后跑 `./3-amcl_test.sh`：`/amcl`、`/map_server`、`/move_base`、`/rplidarNode` 四个都在；**RViz 里激光点贴合地图墙线**（定位准）；`2D Nav Goal` 的红色 `Full Plan` **绕开障碍物、不贴墙**。⇒ **D9 安全隐患正式关闭**（代码 2026-10-10 已注释掉那三行，本次是上机证实它真的生效）。<br>⚠️ **但还没验的是"机器人是否真的按这条路走过去"** —— `Full Plan` 绕障只是**规划层**的结论；局部规划器、代价地图、`cmd_vel` 这一段尚未实跑验证。**自跑前应先单独做一次纯自主导航**（`2D Nav Goal` 后看它真的动、真的绕、真的到）。 |
| **B1 根因查清 ✅（不阻塞自跑）** | RViz `view_navigation.rviz:415` 的 `Image Topic` 是 `/usb_cam2/image_raw`（由 `yolov8_ros` 的 `usb_cam2` 节点发，读 `/dev/video2`），而 `3-amcl_test.sh` 起的是 `cam_track` 的 `usb_cam` 节点（读 `/dev/video0`，发 `/usb_cam/image_raw`）—— **两台不同的相机**。所以 `3-amcl_test.sh` 会话里 RViz 永远 `No Image`，**不是相机坏**。<br>`grasp.sh` 里 `multi_yolo` 起的正是 `usb_cam2`，与 RViz 配置自动对上。**剩下只差确认 video2 那台是否对着工作台**（跑 `grasp.sh` 看画面即可）。 |

**第 3 步 · 导航回归 + D9 避障验证 ⭐ 最优先 —— ✅ 2026-10-12 已通过**

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

**✅ 实测结果**（详见 `docs/上机操作清单.md` 进度记录表）：四个节点齐、激光贴墙线、
`Full Plan` 绕障不贴墙 ⇒ **D9 关闭**。

---

### ⏭️ 下一步：让它自己跑一遍（2026-10-12 用户提出）

**先做「纯自主导航」这个增量，再上 `grasp.sh` 完整流程。** 理由：第 3 步只验了
**规划层**（红色 `Full Plan` 绕障），**局部规划器 → 代价地图 → `cmd_vel` → 底盘真动**
这一段从没实跑过。直接上 `grasp.sh` 的话，万一它不动，你分不清是导航的问题
还是 YOLO/抓取的问题。分层上，一次只引入一个新变量。

**① 清场 + 前置检查**

```bash
cd /media/abot/999/abot
./killall.sh                       # 一切换脚本就先杀干净（旧版杀不干净会让雷达起不来）
ls -l /dev/ttyUSB*                 # 必须看到 ttyUSB0/1/2 —— 见 C7
ls /sys/bus/usb-serial/devices/    # 不能为空
```

**② 起导航**（`3-amcl_test.sh`：导航 + 相机 + RViz + 键盘 + pub_point，**不含机械臂**）

```bash
./3-amcl_test.sh
rosnode list | grep -E "amcl|map_server|move_base|rplidarNode"
```

**③ RViz 里必须先给 `2D Pose Estimate`** —— ★ **每一次 AMCL 启动都要做**，不做的话
AMCL 不知道机器人在哪，`2D Nav Goal` 给出去它也不知道往哪走。

**④ 给一个 `2D Nav Goal`，然后盯着看这三件事**（这是本次真正要验的）

| 看什么 | 在哪看 | 通过标准 |
|---|---|---|
| 它**真的动了** | 地板 / 轮子 | 不是只画了条线就不动 |
| 它**绕开障碍**了 | 实物 | 路上放个箱子，它绕过去而不是推着走 |
| 它**停到点**了 | RViz 里的机器人位姿 | 到位后自己停住，不再抽动 |

**⑤ 出问题怎么读**：`3-amcl_test.sh` 那个标签页里 `move_base` 的输出是关键。
- **`Full Plan` 有、局部路径没有** → 局部规划器（`DWAPlannerROS`）或代价地图问题
- **两条路径都有、底盘不动** → 看 `rostopic echo /cmd_vel` 有没有数；有数不动是底盘/串口（C7）
- **定位在飘** → 回到 `2D Pose Estimate`，或确认用的是 `my_home` 这张图

**⑥ 全过之后**，再上 `./grasp.sh`（导航 + YOLO + 抓取 + 放置 + RViz）。
这时再看 RViz 的 `Image` 面板 —— 按 B1 的结论它订阅的是 `/usb_cam2/image_raw`，
`grasp.sh` 起的 `multi_yolo` 正好发这个话题，**应该直接有画面**。
画面里**能看到工作台和机械臂前方**，B1 就彻底闭上了；看不到就是 video2 那台相机
**装的朝向不对**，那是装配问题，不是代码问题。

**安全**（用户已确认要自跑，把这几条摆明）：
- 场地清空，人**别站在机器人路径上**
- **手别伸进机械臂范围** —— `grasp.sh` 会动机械臂，它不看人
- 手边留着 `Ctrl-C`，或者直接 `./killall.sh`
- `2D Nav Goal` 先给**近的、直的**点（比如 1 米正前方），确认能动再给远的、要绕弯的

**之后**：第 4 步正式结掉 B1、第 5 步再建图就用同一套流程、第 6 步清 D 系列
（D13/D14 代码已改，但**尚未上机验证**）。

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

### 晚间那一批（D9 补修 + B1 真因修复；**✅ 已同步到 U 盘**）

| 文件 | 改动 | 在 U 盘上？ |
|---|---|---|
| `abot/src/robot_slam/scripts/nav.py` | **D9 补修**：注释掉等价的 3 行 `update_configuration({"enabled": 0})`（旧行号 201–203），并写下危害说明（`grasp.sh`/`4-test.sh` 经 `grab.launch` 启动它 + 5 处盲退 + 写死的厂商点位） | ✅ |
| `abot/src/robot_slam/scripts/vl_nav.py` | 同上（旧行号 199–201）。该文件**无任何 launch/sh/文档引用**，属顺手拆雷 | ✅ |
| `abot/src/yolov8_ros/launch/usb_cam.launch` | **B1 尝试修复**：加 `<param name="framerate" value="25" />`（相机 640×480 只到 25 fps，`usb_cam` 默认要 30 ⇒ STREAMON EIO）。**❌ 上机验证失败**，见下节 | ✅（25 fps 那版） |
| `docs/上机调试问题汇总.md` | D9 表格行改写为"三处文件" + 详解段补记；新增「B1 续」小节（真因、实测表格、已排除项、后备方案） | — 文档只在 D 盘 |
| `docs/上机操作清单.md` | 新增「第 3.5 步 纯自主导航实跑」；第 4 步 B1 改写为真因结论；第 3.5 步（6）加 `grasp.sh` 剧本警告；进度表更新 | — 同上 |
| `handoff_2026-10-12_onboard-checkup.md` | 本文件 | — 同上 |

**还原方式**：三个源码文件里把改的东西去掉/`#` 去掉即可（就是恢复厂商原行为）。

### 第三批 · 2026-10-12 17:3x（B1 帧率假设被推翻；**⚠️ 未同步 U 盘**）

**用户澄清了一件事**：他说的"导航"是指 **`nav.py` 那条写死路线跑全程**（`grasp.sh` 第 7 个标签页），
不是手动给一个 `2D Nav Goal`。⇒ 目标本身没变，但**全程自跑现在有两个硬阻塞**：

1. **相机不活 ⇒ 卡在第一个抓取点。** `nav.py` 每到抓/放位就是 `while True` 死等 `/success`，
   而 `grab.py` **只在五步抓取全走完（第 193 行）才 `set_param('success', True)`**，
   那五步靠相机 + YOLO。相机死 ⇒ 永远没有 `/success` ⇒ 机器人停在原地、不报错、不退出。
2. **`grab.launch` 里 16 个坐标是厂商实验室的**（3×3 货架网格 + 接近点），不是本房间的。

⇒ 顺序只能是 **① 相机 → ② 量点位 → ③ 全程**。`3-amcl_test.sh` 那条"手给点"的路
可作 ① 之前的导航链路验证（第 3.5 步）。

**B1 新进展 —— 帧率假设 ❌ 已推翻：**

`framerate=25` 确实生效了（日志变 `at 640x480 via mmap (yuyv) at 25 FPS`），
**但下一行照样 `VIDIOC_STREAMON error 5, Input/output error` + `process has died`**。
⇒ **参数生效、故障不变 ⇒ 帧率不是元凶。**

**⭐ 决定性取证已做（17:5x）—— 真因定性为「设备/内核层」，与 `usb_cam` 配置无关：**

```
v4l2-ctl -d 2 --stream-mmap --stream-count=10      ⇒ Input/output error
```

这条命令**不带任何参数、绕开 `usb_cam`**，就是让设备用自己的默认设置推 10 帧。
**连它都 EIO ⇒ 帧率和像素格式都不可能是元凶。** 对照：

| 谁 | 结果 |
|---|---|
| `usb_cam2` 节点（yuyv / 25 fps） | `VIDIOC_STREAMON error 5` |
| `v4l2-ctl`（**无参**） | `Input/output error` |
| `VIDIOC_S_FMT` 设格式 | **成功** —— 只有"开流"这一步过不去 |

⇒ 和 **C7 同一族**（USB 设备掉线、没干净重枚举）。**`sudo reboot` 对症，17:5x 用户已执行。**

> **`pixel_format` 改 `mjpeg` 因此降级为"顺手买的带宽保险"，不是根因修复** ——
> 但仍保留（仓库已改），因为两台相机同挂 `usb-3610000.xhci`，改完顺带把带宽余量也留出来。
> ⚠️ 那行命令的 `-d` **不能省**，写成 `v4l2-ctl dev/video2 ...` 会报 `unknown arguments: dev/video2`
> 并吐一屏帮助（用户已踩过一次）。

**⭐ 结果（2026-10-12 17:50）：相机已出图 —— B1 主体解决。**

`sudo reboot` + `pixel_format=mjpeg` 之后跑 `./grasp.sh`，
第 4 个标签页（`multi_yolo`）**不再报 `STREAMON error 5`、进程不再退出**，
**RViz 的 `Image` 面板出画面了**（能看到地面和一条亮的东西）。

日志变成 mjpeg 解码路径的常态输出：

```
swscaler: No accelerated colorspace conversion found from yuv422p to rgb24
mjpeg:    unable to decode APP fields: Invalid data found when processing input
```

**⚠️ 这两行是警告不是错误** —— `usb_cam` + MJPG 一开就刷屏，别被吓到。
判据：**图出来了 ⇒ 红字是噪音**；致命的只有 `VIDIOC_STREAMON error 5` / `process has died`。
`yuv422p → rgb24` 这个转换**只在 mjpeg 解码分支出现** ⇒ 证明改的配置确实生效了。

**⚠️ 归因未分离**：这次同时改了两个变量（`sudo reboot` + `mjpeg`），
**分不清是谁治好的**。想分离就补跑 `v4l2-ctl -d 2 --stream-mmap --stream-count=10`
（该命令走设备默认格式、不经 mjpeg）：能推流 = 重启治好的；仍 EIO = mjpeg 治好的。
**不阻塞使用，只是留个记录。**

**☐ 唯一剩余项 —— 画面拍到的是什么？**

| 画面里看到 | 结论 |
|---|---|
| **工作台 / 机械臂正前方 / 桌上要抓的东西** | ✅ **B1 彻底关闭** |
| 地板 / 天花板 / 墙 | ⚠️ video2 那台**相机装配朝向不对** —— **硬件问题，不是代码问题**，得找厂商 |

**若以后复发（重启仍 EIO）** ⇒ 才轮到硬件/常驻配置：
换 USB 线/口（避开 video0 那个控制器）→ 只插一台试 → `dmesg | grep -i uvc` → 最后才怀疑相机。

| 文件 | 17:3x 改动 | 在 U 盘上？ |
|---|---|---|
| `abot/src/yolov8_ros/launch/usb_cam.launch` | `pixel_format` 改 `mjpeg`，注释块重写（记录帧率假设被推翻的完整取证过程） | **✅ 已同步（2026-10-12 18:0x，U 盘插回 Windows 后）**：改前盘上 md5 `10766c488077bfdaa646090a6c3d050a`（25 fps + `yuyv`），改后两侧同为 `11292d9f70fb7241602518c641987c70`。`diff -rq` 全树复查：**除 `__pycache__` 外零差异** |
| `docs/上机调试问题汇总.md` | B1 续小节重写：标注帧率假设已推翻、格式假设待验、补 `v4l2-ctl -d` 取证命令与 EIO 判读 | — 文档只在 D 盘 |
| `docs/上机操作清单.md` | 第 4 步标题改「🔶 表因已清、真因仍在查」；补 (A)(B)(C)(D) 四步判读表；第 3.5 步开头加「"自己跑一遍"两个意思」对照表；进度表两行更新 | — 同上 |
| `docs/功能测试清单.md` | 末表 B1 行改掉"已修"的错误结论 | — 同上 |

> **同步欠账已清**。`docs/` 不用管 —— 它从来就不在 U 盘上。
> **另**：`nav.py` / `vl_nav.py` 在 17:2x 就同步过了，本次全树 `diff -rq` 确认仍是同步的，
> 没被这次重启/拔插弄乱。

### 会话收尾状态（2026-10-12 18:1x）

**本轮真正解决的两件事：**

| # | 事 | 结果 |
|---|---|---|
| 1 | **B1 相机** | ✅ **修好了**。`sudo reboot` + `pixel_format: yuyv → mjpeg` 之后 RViz `Image` 面板出画面。日志里 `No accelerated colorspace conversion` / `unable to decode APP fields` 是**警告非错误**（判据：图出来了就是噪音）。⚠️ **归因未分离**（reboot 与 mjpeg 同时改），想分离就补跑 `v4l2-ctl -d 2 --stream-mmap --stream-count=10`，**不阻塞使用**。 |
| 2 | **C9 重启后雷达没起来** | ✅ **已恢复**。串口侧完全正常（`/dev/rplidar -> ttyUSB0`、三个 ttyUSB 全在）⇒ 是 `rplidarNode` **抢在 udev 建软链接之前**去开串口而退出。`./killall.sh` + `./grasp.sh` 重开一次即好 ⇒ **定性为时序/竞态，不是配置或硬件**。完整记录见 `docs/上机调试问题汇总.md` **C9**。用户反馈"有红框了"。 |

**⭐ 本轮新学的两条通用规律（已写进 `docs/上机调试问题汇总.md`）：**

1. **时好时坏 = 时序；每次必坏 = 配置/硬件。** ⇒ C9 属前者，能从时序上治。
2. **"重开一次就好"不是万能药** —— 它治"进程死了"，治不了"设备真没回来"
   （后者软链接会悬空，重开多少次都不行，得拔插 USB / 再重启）。
   根子上仍是 C7 那条铁律：**设备回来 ≠ 进程自愈**。

**新规程（开机时照做）：**

1. 开机后**等 30～60 秒**（等 udev 建完设备），别抢跑
2. 先验设备：`ls -l /dev/rplidar /dev/abot_driver` + `ls /dev/ttyUSB*`（三个都要在、不悬空）
3. 再 `./grasp.sh`
4. 起来后**第一件事** `rostopic hz /scan` —— 出 `average rate: 10` 才算真起来了

**⚠️ 会话结束时的机器人状态**：`grasp.sh` 全套在跑，`nav.py`（`/navigation_node`）
停在 `input("Please enter 1 to continue: ")`。
**没有按过 `1`** —— 即从未往厂商的 16 个写死点位跑过。机器人安全、未移动。

**未同步项**：`docs/` 只在 D 盘（历来如此）；`abot/` 源码与 U 盘 `diff -rq` 零差异。

**下一次开工的入口 —— 用户的真实目标仍然是"让它自己按路线跑一遍全程"，两个前置：**

| 前置 | 状态 |
|---|---|
| ① 相机不活 ⇒ 卡在第一个抓取点（`nav.py` 的 `while True` 死等 `/success`，`grab.py:193` 只在五步抓取全走完后才 `set_param('success', True)`） | ✅ **相机已修**，这一环通了 |
| ② `grab.launch` 的 16 个点位是厂商实验室的（3×3 货架网格 + 接近点） | ❌ **仍待办** —— 要在本房间重新量 |

⇒ **下一步顺序：① 先跑第 3.5 步（`3-amcl_test.sh` + `2D Nav Goal` 手给近点）验"局部规划器 → 底盘真动"这一段（从未实跑过）→ ② 再量 16 个点位 → ③ 才上全程。**

---

### 下次开工 SOP（照做，别跳步）

```
① 机器人开机后，先等 30～60 秒          ← 等 udev 建完设备，别抢跑（C9 的教训）
② 验设备（两条）
     ls -l /dev/rplidar /dev/abot_driver
     ls /dev/ttyUSB*                      ← 三个都要在，软链接不能悬空
③ cd /media/abot/999/abot
   ./killall.sh                           ← 清残留进程（C3/D10）
   ./3-amcl_test.sh                       ← 只上导航，不带机械臂，最安全的增量
④ 起来后第一件事验雷达
     rostopic hz /scan                    ← 出 average rate: 10 才算真起来了
⑤ RViz 里给 2D Pose Estimate              ← 每次 AMCL 启动都必须做
⑥ 给一个近的、直的 2D Nav Goal，盯三件事：
     它真的动了？ 绕开障碍了？ 停到点了？   ← ★ 第 3.5 步，这段从未实跑过
⑦ ⑥ 过了才谈得上量点位、才谈得上全程
```

**安全红线（任何时候都成立）：**

- `/scan` 没数据时，**绝不能**按 `grasp.sh` 第 7 个标签页那个 `1`
  （代价地图是空的 = 闭着眼往写死的坐标跑）
- `nav.py` 有 **5 处 `navi.end()` 盲退**（`linear.x=-0.3` 连发 0.9 秒 ≈ 0.27 米，不看传感器）
  —— 数值 2026-10-15 已更正，见本文件第三节之二第 158 行
- 要动机械臂时，人别站在臂展范围内
