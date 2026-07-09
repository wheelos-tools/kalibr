# 相机间标定（Camera-to-Camera）—— 详细指南

> English version: [`CAMERA2CAMERA_CALIBRATION_GUIDE.md`](CAMERA2CAMERA_CALIBRATION_GUIDE.md)

用 Kalibr + AprilGrid 标定两个视场重叠的 RTSP 相机之间的相对位姿（**外参**），
覆盖完整流程：

1. [安装并配置环境](#1-安装并配置环境)
2. [采集数据](#2-采集数据)
3. [把数据转换成标定所需的格式](#3-把数据转换成标定所需的格式)
4. [标定](#4-标定)
5. [评估标定结果](#5-评估标定结果)

这是**成对**（两相机）标定流程 —— 也是最基本的单元。要把多对拼成一个 N 相机相机组
（如 `68→66→64→65→67`），先在此处逐对标定，再按
[`EXTRINSIC_CALIBRATION_GUIDE_zh.md`](EXTRINSIC_CALIBRATION_GUIDE_zh.md)
做联合求解。

以下所有命令都在 **`extrinsics/` 目录下**执行；脚本位于 `scripts/`，标定板定义为
`aprilgrid_6x6.yaml`。

### 基本概念（先读一遍）

- **要求解什么。** 4×4 刚体变换 `T_cn_cnm1`，把三维点从父相机坐标系映射到子相机坐标系
  —— 即子相机相对父相机的位置 + 朝向。Kalibr 会在同一次优化中一并（重新）估计每个相机的
  内参。
- **为什么用 AprilGrid 而不是棋盘格。** 每个 tag 都有唯一 ID，因此即使只看到**部分**标定板
  也仍然有效。这正是能在相邻相机约 30% 视场重叠下工作的原因 —— 棋盘格必须被完整看到。
- **为什么不需要硬件同步。** 相机是各自独立的 RTSP 流（抖动 50–150 ms）。外参是**静态**
  变换，所以我们在每个位姿处让标定板**保持静止**，并给同一位姿的两帧打上**完全相同的合成
  时间戳**；Kalibr 便会在 `--approx-sync 0.01` 下把它们融合（见第 2–3 步）。
- **⚠️ 检测器注意事项。** 通用 Python AprilTag 检测器（`pupil-apriltags`、OpenCV `aruco`）
  **无法**识别这块板；**Kalibr 自带的检测器可以**（结果达到亚像素精度）。请通过实际运行
  Kalibr 来确认能否检测，而非用 Python 脚本验证。

---

## 1. 安装并配置环境

涉及两套工具链：

- **Kalibr** —— 在 **Docker** 容器（ROS Noetic）中运行，用于第 4 步。
- **Python 3** 辅助脚本（提取 / 转换 / 评估）—— 在**宿主机**上运行。

### 1a. Kalibr（Docker）

仓库自带 Dockerfile（`Dockerfile_ros1_20_04`，ROS Noetic + catkin 编译的 Kalibr）。
在仓库根目录构建一次镜像：

```bash
cd /path/to/kalibr            # 仓库根目录（extrinsics/ 的上级）
docker build -t kalibr -f Dockerfile_ros1_20_04 .
```

首次构建需 15–40 分钟（要编译 Kalibr）。验证：

```bash
docker images | grep kalibr           # -> kalibr  latest  ...
docker run --rm --entrypoint bash kalibr -c \
  'source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras --help | head -3'
```

说明：
- 镜像默认 `ENTRYPOINT` 会设置 `KALIBR_MANUAL_FOCAL_LENGTH_INIT=1`。我们用 `bash -c`
  覆盖了 entrypoint（第 4 步），并不依赖它 —— AprilGrid 能自动给出较好的焦距初值。若某次
  运行焦距初始化失败，可在 `docker run` 加 `-e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1`。
- 无显示器的宿主机务必加 `--dont-show-report`（第 4 步）。要看 GUI 报告，则转发 X11
  （`-e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw`）并去掉该参数。
- 在 Linux 宿主机上，容器写出的文件属 `root` —— 可给 `docker run` 加
  `--user $(id -u):$(id -g)`，或事后 `chown`。

### 1b. Python 辅助脚本（宿主机）

Python 3.8+，外加三个包：

```bash
python3 -m pip install --user "numpy>=1.21" opencv-python pyyaml
python3 -c "import cv2, numpy, yaml; print('cv2', cv2.__version__, '| numpy', numpy.__version__)"
```

> **已知坑。** 损坏的用户级 `numpy` 可能能 import 却在 `numpy.ma` 内崩溃（如 `np.median`
> → `ImportError: cannot import name 'core'`）。修复：
> `python3 -m pip install --user --force-reinstall --no-cache-dir numpy`。

### 1c. 前置条件清单

- [ ] `kalibr` Docker 镜像已构建（1a）。
- [ ] 宿主机 Python 及 `opencv-python`、`numpy`、`pyyaml`（1b）。
- [ ] 已打印物理 **AprilGrid**（`april_6x6_80x80cm_A0.pdf`）；其几何与 `aprilgrid_6x6.yaml`
      一致（6×6 tags，tagSize 0.088 m，tagSpacing 0.3）。
- [ ] *（可选）* 各相机的棋盘格内参 `../intrinsics/intrinsics_cam*.yaml` —— 仅
      [第 5 步](#5-评估标定结果)的交叉核对需要。

---

## 2. 采集数据

目标：录到 AprilGrid 在**两个相机**中都可见、且位姿分布充分的素材。

### 操作规范

- 让标定板始终处于该对的**重叠区域**（两相机同时可见）。
- **走走停停**：移动 → **静止保持 3–5 秒** → 再移动。这种静止正是我们无需硬件同步的前提。
- **约 20–30 个不同位姿**，覆盖：
  - **距离** —— 近 / 中 / 远（标定板约占画面 10–40%），
  - **倾角** —— 让标定板偏离正对平面，做俯仰与偏转，
  - **位置** —— 在重叠带内扫过（左/中/右、高/低）。
- 避免运动模糊；标定板要足够大，使 tag 清晰。
- 只有正对或只在单一距离的位姿会使内参约束不足 —— **多样性才能让求解适定。**

### 两种采集方式

**A. 每个相机录一段视频（推荐）。** 同一时刻启动该对的两个相机（使时钟接近）。每个相机一个
视频文件；相机 id 由文件名中的 IP 解析。以 `64_65` 为例：

```
c2c_videos/
└── 64_65/
    ├── cam2_192.168.1.64_00000.mkv
    └── cam3_192.168.1.65_00000.mkv
```

**B. 实时同步快照。** 若能实时访问相机，可同时从两个相机各抓一帧并打上相同时间戳（每次抓取
时保持标定板静止）：

```bash
python3 scripts/capture_sync_rtsp.py --out pairs/64_65 --interval 0.5   # 或 --manual
```

方式 B 直接写出第 3 步的文件夹结构 —— **跳过第 3 步**，直接进第 4 步。

---

## 3. 把数据转换成标定所需的格式

Kalibr 读取 ROS **bag**；分两小步到达。

### 3a. 视频 → 时间戳命名的图像对

`extract_pairs_from_video.py` 把每对视频转成去重、共享时间戳的 PNG。它给每帧打分
（48×48 对比度归一化签名），把每个 3–5 秒的低运动段视为一个**位姿**，保留其中**最清晰**的一帧，
并给同一位姿的两个相机打上相同的合成时间戳。它还会对两路运动信号做互相关，恢复相机间的时间
偏移。

```bash
# 先用 montage 调参 —— 只写审阅图 + csv，不写 PNG：
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --dry-run

# 然后提取（处理该根目录下找到的所有对）：
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --prune-unpaired
```

每对的输出：
```
pairs/64_65/
├── cam64/<ts_ns>.png     # 两个相机使用相同的 <ts_ns> 文件名前缀
├── cam65/<ts_ns>.png
├── review_montage.jpg    # 每个位姿左右并排 cam64 | cam65 —— 请检查这张图
└── index.csv
```

**剔除不良位姿：** 打开 `review_montage.jpg`；对任何模糊、标定板未同时出现在两相机、或冗余的
位姿，把其 `<ts>.png` 从**两个相机文件夹中都删除**（前缀相同）。可调参数：`--min-still`
（停留秒数，默认 1.0）、`--dup-thresh`（合并近似位姿）、`--per-pose N`（每段保留 N 张最清晰）、
`--still-thresh`（覆盖自动运动阈值）。

> 只处理一对？`--pair ../c2c_videos/64_65 --out pairs`。

### 3b. 图像对 → ROS bag（在 Docker 内）

```bash
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/64_65 --output-bag 64_65.bag'
```

19 位纳秒文件名成为 ROS 消息时间戳；每个 `cam<N>/` 文件夹成为 topic `/cam<N>/image_raw`。

---

## 4. 标定

运行双目求解。**topic 必须按链路顺序**（父相机在前），每个相机一个 `pinhole-radtan` 模型。
可把 3b 与本步合并到一次 `docker run`：

```bash
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/64_65 --output-bag 64_65.bag &&
  rosrun kalibr kalibr_calibrate_cameras --bag 64_65.bag \
      --topics /cam64/image_raw /cam65/image_raw \
      --models pinhole-radtan pinhole-radtan \
      --target aprilgrid_6x6.yaml --approx-sync 0.01 --dont-show-report'
```

- `--approx-sync 0.01` 是安全的：同一位姿的两帧时间戳完全相同，相邻位姿间隔很大，视图不会互相
  污染。
- 单对运行时间通常 1–5 分钟。

**输出**（生成在 bag 旁，即 `extrinsics/` 下）：

| 文件 | 内容 |
|---|---|
| `64_65-camchain.yaml` | **标定结果** —— 每个相机的内参 + `T_cn_cnm1`。 |
| `64_65-results-cam.txt` | 重投影误差、参数 ±1σ 标准差、各基线。 |
| `64_65-report-cam.pdf` | 诊断图（重投影散点、角点覆盖）。 |

---

## 5. 评估标定结果

标定结果未经检查绝不能直接使用。以下四项独立检查（由易到难），结果应**全部**通过。

### 5a. 重投影误差（首要指标）

打开 `64_65-results-cam.txt`。每个相机会报告：

```
reprojection error: [mean_x, mean_y] +- [std_x, std_y]     # 像素
```

| 指标 | 良好 | 勉强 | 拒绝 |
|---|---|---|---|
| 单相机重投影 **std**（px） | ≤ 0.3 | 0.3 – 0.5 | > 0.5 |
| 重投影 **mean**（px） | ~0（±0.01） | — | 偏大 / 有偏 |

作为参考，本套设备正常结果约为 **±0.17–0.28 px**。均值应基本为零；非零均值意味着模型存在
系统性失配。

### 5b. 参数不确定度（±1σ）

同一文件在每行 `projection:` / `distortion:` / baseline 之后打印 `+- [...]` —— 即优化器给出的
各参数 1σ 标准差。

- **焦距 / 主点** 的 σ 应为**亚像素**级（< 约 2 px）。σ 偏大 ⇒ 该参数约束不足（通常是位姿太少 /
  多样性不够）。
- **基线平移** 的 σ 应 **≤ 数毫米**。若任一平移分量的 σ > 约 1 cm，则不要信任该基线。

### 5c. 外参的物理合理性

把变换转成可读位姿，并与卷尺实测对照：

```bash
python3 scripts/kalibr_to_opencv.py --camchain 64_65-camchain.yaml
```
每条链路打印：
```
cam64 -> cam65: pan(yaw)=-44.93  pitch=-1.03  roll=+0.60 deg | baseline=15.2 cm
```
核对项：
- **基线**与相机实际间距一致（此处内侧对约 15 cm，外侧对约 27 cm），误差在约 1 cm 内。
- **偏转（yaw）**与安装扇形角一致；水平安装时**俯仰与横滚**接近 0（几度可接受，几十度则说明
  位姿错误）。
- 多相机链路可用 `scripts/extrinsics_relative.py --camchain <file> --ref 64` 列出每个相机相对
  参考相机的角度 —— 应是平滑、单调的扇形分布。

### 5d. 内参与棋盘格交叉核对（可选，但推荐）

Kalibr 仅**基于重叠区域视图重新估计**内参；与独立的棋盘格内参对比可发现过拟合：

```bash
python3 scripts/compare_calib.py --camchain 64_65-camchain.yaml
# -> 64_65-comparison-report.md
```
报告会列出 Kalibr 与棋盘格的 `fx, fy, cx, cy, k1…`，并给出每个参数的相对 Δ。

| Δ（Kalibr vs 棋盘格） | 判定 |
|---|---|
| `fx/fy` 在约 3% 内、`cx/cy` 在约 2% 内 | 一致 —— 良好 |
| 更大，或 `k1` 变号 / `fx` 偏差 >10% | 某一套过拟合 —— 需排查 |

**该用哪套？** Kalibr 的内参与外参自洽（一起重投影更干净），但来自仅重叠区域的视图；棋盘格那套
全画幅覆盖更好。对同时使用内参 + 外参的拼接器，默认用 **Kalibr 的**；仅当你更信任棋盘格覆盖并
接受轻微不一致时才用棋盘格那套。

### 5e. 目视检查（PDF）

浏览 `64_65-report-cam.pdf`：
- **重投影散点**应是紧凑、居中的点云（无结构 / 无弧形）。
- **角点覆盖**图应铺满大部分画面，而非聚在中心 —— 覆盖稀疏之处提示重新采集时该补哪些位姿。

### 若不通过

| 现象 | 解决 |
|---|---|
| 重投影 std > 0.5 px | 位姿太少 / 模糊 → 按第 2 步重新采集，增加多样性。 |
| 某参数 σ 偏大 | 增加不同距离与倾角的位姿；该参数当前约束不足。 |
| 基线 / 角度不合理 | 检查 topic 顺序（父相机在前）以及两相机是否都看到板；重新查看 montage。 |
| Kalibr 报 "0 observations" | 未检测到标定板 —— 确认 `--target` 与实际板一致；必要时用 `kalibr_create_target_pdf` 重生成。 |
| 内参与棋盘格差距大 | 覆盖太薄导致过拟合 —— 补全画幅扫板，或固定内参。 |

---

## 命令速查（以 `64_65` 为例）

```bash
cd extrinsics
# 3. 转换
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --prune-unpaired
# 4. 建 bag + 标定（Docker）
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/64_65 --output-bag 64_65.bag &&
  rosrun kalibr kalibr_calibrate_cameras --bag 64_65.bag \
    --topics /cam64/image_raw /cam65/image_raw \
    --models pinhole-radtan pinhole-radtan \
    --target aprilgrid_6x6.yaml --approx-sync 0.01 --dont-show-report'
# 5. 评估
python3 scripts/kalibr_to_opencv.py --camchain 64_65-camchain.yaml
python3 scripts/compare_calib.py   --camchain 64_65-camchain.yaml
```

录制视频、提取出的图像、bag 以及 Kalibr 输出都已被 **gitignore** —— 属于每套设备各自的数据，
运行本指南即可重新生成。仓库跟踪的是方法本身（本指南 + `scripts/`）。
