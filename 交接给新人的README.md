# 交接给新接手的人 —— 先读这一页

> 这个压缩包里是一个 ROS 1 机器人项目。**代码在 PC 上跑不了**，它要跑在机器人上。
> 别急着敲命令，先花 3 分钟看完这一页。

---

## 1. 这是三样东西，不是一样

| | 在哪 | 说明 |
|---|---|---|
| **代码仓库** | 这个压缩包（或 GitHub `ano521/robot-car`） | 你在 PC 上用 Codex 改这里 |
| **工作区 = 一块 U 盘** | 车上 `/media/abot/999/abot` | 机器人实际运行的就是这块盘上的文件 |
| **机器人** | Jetson，Ubuntu 20.04 + ROS Noetic，登录 `abot@abot` | 只有它能跑 ROS 和机械臂 |

**PC 上的改动要手动拷到 U 盘，U 盘改的又要拷回 PC** —— 没有自动同步。
这个项目就是在这上面栽过两次（车上改了两天、仓库没跟上，导致后面一整轮排查全建立在旧代码上）。

## 2. 起手按这个顺序读

1. **`abot/AGENTS.md`** ← Codex 会自动读它，人也要读，这是最短的硬约束
2. **`abot/交接日志-2026-10-28.md`** ← **最新事实**，第 8 节讲为什么换了视觉方案，第 9 节是当前进度
3. `abot/交接日志-2026-10-26.md` ← 抓取/标定/模板这一整套是怎么来的
4. `docs/抓取功能/05-零基础上机操作.md` ← 第一次上机照着做
5. `docs/上机调试问题汇总.md` ← 编号问题总账（A/B/C/D），改之前先查，很多"像 bug"的其实是已查清的环境问题

## 3. 当前进度（2026-10-28 深夜）

**已经成立：**

- 视觉链路 `template_orb → /objects → ai_locate → /yolov8_detections → grasp_once` **全线打通，25 Hz**
- 墙上/大张的模板图**识别很准**
- 机械臂串口 `/dev/arm` 权限正常（mode 666），`a.py` 能控 6 个舵机

**还没成立（按顺序）：**

1. **从未标定** —— `config/grasp_calib.json` 不存在，现在用的是厂商的 4 点插值
2. **小物块认不出** —— 大概率是失焦（大目标准、小的糊）
3. `video0` 配 `mjpeg` 取不到帧（它历史上用 `yuyv`；`video2` 配 mjpeg 正常）

## 4. 三条红线（违反会烧钱或伤人）

1. **标定完成之前，禁止 `dry_run=false` 抓取。** 现在臂会伸到"它以为该去的地方"，
   而不是物块所在的位置 —— 这种失败最迷惑人。
2. **不改车内的 `/etc` 和 `/home/abot`**（含 udev 规则、`.bashrc`）。
   所有改动只落在 U 盘工作区里。
3. **三条视觉链不能同时起**（`grasp.sh` / `find_object_2d` / `template_orb` 都占 `/usb_cam2` 和 `/objects`，
   而且一台相机同一时刻只能被一个节点打开）。

## 5. 环境事实（背下来能省很多时间）

```bash
cd /media/abot/999/abot

# ★ 每开一个新终端，敲 ROS 命令之前必须 source 两个：
source devel/setup.bash     # 否则 rospack 找不到 yolov8_ros，roslaunch 报 "neither a launch file"
source ros-env.sh           # 否则命令无限卡住且不报错（环境里预置了旧热点的死 IP）

# 串口别名（认的是 USB 物理插口，别换插口；也别硬编码 ttyUSB 编号）
ls -l /dev/abot_driver /dev/rplidar /dev/arm
```

| 项 | 值 |
|---|---|
| 底盘串口 | `/dev/abot_driver` |
| 雷达串口 | `/dev/rplidar` |
| 机械臂串口 | `/dev/arm`（mode 666，谁都能开）|
| 相机 | 两台：`video0`（机械臂上，拍近）、`video2`（前置，拍远）；**每台有两个节点，真正可用的是 video0 / video2** |
| 启动脚本 | `./ai.sh`（视觉链）、`./3-amcl_test.sh`（导航，最安全）、`./grasp.sh`（全套，会动臂）、`./killall.sh`（清场）|

## 6. 最短的上手命令

```bash
cd /media/abot/999/abot
./killall.sh                        # 清残留（切换前必做）
./7-grasp-check.sh | grep '!!'      # 只读体检，没有输出=全绿
./ai.sh dry orb 2>&1 | grep -v swscaler   # 视觉链（dry=臂不动）
```

## 7. 包里**没有**的东西（重要）

- **模型权重**（`*.pt` / `*.bin`，约 880 MB）—— 抓取链路用不到；需要语音/TTS/whisper 时从车上那块 U 盘拷
- **`build/` / `devel/`** —— 和具体机器绑定，到新机器上要自己 `catkin_make`
- **运行日志、备份目录**

## 8. 如果要用一块**新的** U 盘

工作区路径 `/media/abot/999/abot` **被硬编码在几十个脚本里**。新盘必须满足：

- 卷标 `999`
- 挂载到 `/media/abot/999/abot`
- 用户 `abot`

否则一堆脚本会找不到文件。

---

*这份 README 只求让对方三分钟内不出错。细节全在 `abot/AGENTS.md` 和两份交接日志里。*
