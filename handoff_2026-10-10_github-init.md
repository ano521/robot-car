# 交接：robot car 工程首次入库 GitHub

日期：2026-10-10
仓库：https://github.com/ano521/robot-car （**私有**，默认分支 `main`）
首个提交：`e75ffdb` — 965 文件 / 374,174 行 / 28 MB

---

## 一、本次做了什么

把 `D:\Program Files\robot car` 下的 ABOT 移动抓取机器人工程（ROS1 catkin 工作空间，
25 个功能包）首次提交并推送到 GitHub。

工作分四步：

1. **扫描工程** — 确认结构：底盘导航、视觉感知、语音交互、抓取执行四大模块
2. **筛选提交内容** — 原目录 1.2 GB，其中绝大部分不能进 GitHub
3. **处理两个嵌套仓库** — `abot_bringup` / `abot_mediapipe` 的特殊情况（见第三节）
4. **提交并推送** — 对齐分支名、修正作者身份、推送

**未改动任何源码文件**（用户明确要求）。仓库文件树哈希在修正作者前后一致。

---

## 二、排除在版本控制之外的内容

`.gitignore` 排除了以下内容，克隆后**需要自行补齐**：

| 排除内容 | 原因 | 体积 |
|---|---|---|
| `robot_audio/paraformer-zh/model.pt` | 单文件 840 MB，超 GitHub 100 MB 硬限制 | 840 M |
| `whisper/whisper-tiny-zh-ct2-int8/model.bin` | 模型权重 | 39 M |
| `build/` `devel/` `install/` `log/` | ROS 编译产物 | ~130 M |
| 中文字体（simsun/SimHei/KaiTi 等） | 微软/苹果专有授权，不可再分发 | ~44 M |
| `sam` `track` `region_counter` 权重 | 模型二进制 | ~17 M |
| `__pycache__`、`*.zip`、运行输出 | 构建/运行产物 | — |

**克隆后的补齐清单**：

- ASR 模型 → `abot/src/robot_audio/paraformer-zh/`
- whisper 模型 → `abot/src/whisper/whisper-tiny-zh-ct2-int8/`
- 分割/跟踪权重 → 各包 `model/` 目录
- 字体 → `abot/src/ocr_detect/ChineseFonts/`、`abot/src/vl_locate/config/SimHei.ttf`
- YOLOv8 权重 → 代码里写死 `/home/abot/yolov8/yolov8n.pt`

---

## 三、两个嵌套 git 仓库的处理（重要）

`abot/src/abot_bringup` 和 `abot/src/abot_mediapipe` 原本是**独立的 git 仓库**
（Gitee 克隆），且内部有**未提交的本地改动**。外层仓库会把它们记成裸 gitlink，
直接提交的话克隆下来这两个目录是空的、改动全丢。

处理方式：把它们的 `.git` 目录**移出**（未删除，可逆）到：

```
C:\Users\lenovo\Desktop\robot-car-nested-git-backup\
    ├── abot_bringup.git      ← 上游 gitee.com/pibot/pibot_bringup (master)
    └── abot_mediapipe.git    ← 上游 gitee.com/rikibot/rikibot_mediapipe (abot 分支)
```

移动后两者成为普通源码目录，本地改动完整保留并已进入 GitHub。

- **想恢复独立仓库**（例如再 pull 上游）：把对应 `.git` 移回原目录
- **想彻底放弃嵌套历史**：删掉桌面备份目录

⚠️ 现在这两个包**不能**再 `git pull` 上游，因为本仓库里它们是普通文件。

---

## 四、新增的两个文件

**`.gitattributes`**（新建）
你在 Windows 编辑、机器人上跑 Linux。本机 `core.autocrlf=true`，不加这个的话
`.sh` / `.launch` 会被提交成 CRLF，到 Linux 上直接报 `bad interpreter: ^M`。
已强制源码用 LF 检出。

**`.gitignore`**（重写，原文件是工具自动生成的）
原内容 7 条规则全部保留，新增编译产物、模型权重、字体、运行输出等规则。

---

## 五、已知问题 / 建议的后续工作

1. **绝对路径硬编码** — 源码多处写死 `/home/abot/...`
   （如 `yolov8_ros/script/multi_yolo.py` 的模型路径、`vl_locate/scripts/server.py`
   的图片与字体路径）。换机器部署需逐一修改，建议改为 ROS param 或 `rospkg` 相对路径。

2. **默认地图不一致** — `robot_slam/launch/navigation.launch` 默认 `my_lab.yaml`，
   而 `abot/maps/` 下只有 `my_room.pgm`，两份地图不同步。

3. **中文文件名** — 存在 `amcl.launch（复件）.xml`、`box_filter（复件）.yaml` 等
   自动生成的"复件"文件，跨平台易出问题。`.gitignore` 已排除新增的此类文件，
   但已入库的部分建议清理。

4. **仓库无 README** — 建议补一份，写明依赖安装、模型下载位置和启动流程
   （`abot/1-gmapping.sh` 等脚本展示了标准的启动顺序）。

5. **API Key** — `vl_locate/scripts/server.py` 调用阿里 dashscope，目前
   `api_key=''` 为空。**不要**把 key 硬编码进去，用环境变量。

---

## 六、快速验证当前状态

```bash
cd "d:/Program Files/robot car"
git log --oneline -1          # e75ffdb
git status -sb                # ## main...origin/main（无分叉）
git ls-files | wc -l          # 965
```
