# 外参标定 —— 5 路 RTSP 相机（Kalibr）

> English version: [`EXTRINSIC_CALIBRATION_GUIDE.md`](EXTRINSIC_CALIBRATION_GUIDE.md)

用 Kalibr 标定 5 相机环视相机组（链路 `68 → 66 → 64 → 65 → 67`，相邻相机之间约
30% 视场重叠）的外参 —— **全程在本地机器上运行** —— 共四步：

```
 1. 录制 ──▶ 2. 构建图像集 ──▶ 3. 标定（本地 Docker）──▶ 4. 导出 YAML
   成对视频     提取 + 合并         bagcreater + calibrate       opencv / apollo
              → 单个 5 相机文件夹   → *-camchain.yaml
```

以下所有命令都在 **`extrinsics/` 目录下**执行；脚本位于 `scripts/`，Kalibr 标定板
定义文件为 `aprilgrid_6x6.yaml`。

### 为什么不需要硬件同步也能标定

这些相机是各自独立的 RTSP 流，没有硬件触发（网络+解码抖动 50–150 ms）。但外参是一个
**静态变换**，所以我们在每个位姿处把标定板**保持静止**，并给同一位姿下两个相机的图像帧
**打上完全相同的合成时间戳** —— Kalibr 便会在 `--approx-sync 0.01` 下把它们融合成一个
观测视图。AprilGrid 的每个 tag 都有唯一 ID，因此即使在 30% 重叠下只看到**部分**标定板
也仍然有效。

> ⚠️ 通用的 Python AprilTag 检测器（`pupil-apriltags`、OpenCV `aruco`）**无法**识别这块
> 标定板，但 **Kalibr 自带的检测器可以**（标定结果达到亚像素精度）。请通过实际跑一次
> Kalibr 来确认能否检测，而不要用 Python 脚本去验证。

### 前置条件

- 每个相机的**内参**已单独标定好 → `../intrinsics/intrinsics_cam*.yaml`
  （Kalibr 在标外参时仍会重新估计内参，详见[第 4 步](#第-4-步--导出-yaml)）。
- 已安装 **Docker**，并在本地构建好 Kalibr 镜像（见第 3 步）。
- **Python 3**，且已安装 `opencv-python`、`numpy`、`pyyaml`。

---

## 第 1 步 —— 每对相邻相机录制一段视频

5 相机链路共需录制 4 对。每对的两个相机要同时开录（一起启动，使两路时钟尽量接近）。
操作要点：

- 让 AprilGrid 始终处于该对的**重叠区域**内，使**两个相机都能看到**。
- **走走停停**：移动 → **静止保持 3–5 秒** → 再移动。正是这种静止让我们无需硬件同步。
- 每对采集 **约 20–30 个不同位姿**，覆盖不同的**距离、倾角、位置**。
- 避免运动模糊；让标定板在画面中保持足够大。

目录结构（相机 id 由文件名中的 IP 解析得到）：

```
c2c_videos/
├── 66_68/  *_192.168.1.68_*.mkv  *_192.168.1.66_*.mkv
├── 64_66/  *_192.168.1.66_*.mkv  *_192.168.1.64_*.mkv
├── 64_65/  *_192.168.1.64_*.mkv  *_192.168.1.65_*.mkv
└── 65_67/  *_192.168.1.65_*.mkv  *_192.168.1.67_*.mkv
```

*（没有录制设备？可改用 `scripts/capture_sync_rtsp.py` 实时抓取同步快照 ——
见[附录](#附录--实时快照采集)。后续流程完全一致。）*

---

## 第 2 步 —— 构建图像集

从每对视频中**提取去重、共享时间戳**的图像对。脚本会检测每个 3–5 秒的**静止停留段**
（用每帧的特征签名衡量低运动区间），并保留其中**最清晰**的一帧，同时给同一位姿下两个
相机的图像打上相同的时间戳。

```bash
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --prune-unpaired
```

打开每个 `pairs/<pair>/review_montage.jpg`（每个位姿左右并排显示 cam A | cam B），把不好的
位姿对应的 `<ts>.png` 从**两个相机文件夹中都删除**。可调参数：`--min-still`、`--dup-thresh`、
`--per-pose`、`--still-thresh`（或先用 `--dry-run` 调试阈值）。

把全部 4 对**合并**成一个多相机文件夹，以便做一次**联合标定**。由于各对是分开录制的，
共享的"桥接"相机（66、64、65）会复用相同的时间戳；通过给每个来源加一个时间偏移，使每次
采集落在各自不重叠的时间窗内，桥接相机便把整条链路串联起来：

```bash
python3 scripts/merge_pairs.py --out pairs/68_66_64_65_67 \
  --source pairs/66_68 0 --source pairs/64_66 10 \
  --source pairs/64_65 20 --source pairs/65_67 30
```

---

## 第 3 步 —— 用 Kalibr 标定（本地 Docker）

先构建一次 Kalibr 镜像（在仓库根目录执行 —— `Dockerfile_ros1_20_04` 随本仓库提供）：

```bash
( cd .. && docker build -t kalibr -f Dockerfile_ros1_20_04 . )
```

然后在**一次本地容器运行**中完成建 bag 与标定，把 `extrinsics/` 挂载为 `/data`
（topic 按链路顺序排列，每个相机一个 `pinhole-radtan` 模型）：

```bash
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/68_66_64_65_67 \
      --output-bag 68_66_64_65_67.bag &&
  rosrun kalibr kalibr_calibrate_cameras --bag 68_66_64_65_67.bag \
      --topics /cam68/image_raw /cam66/image_raw /cam64/image_raw /cam65/image_raw /cam67/image_raw \
      --models pinhole-radtan pinhole-radtan pinhole-radtan pinhole-radtan pinhole-radtan \
      --target aprilgrid_6x6.yaml --approx-sync 0.01 --dont-show-report'
```

`--approx-sync 0.01` 是安全的（同一位姿下各相机时间戳完全相同，相邻位姿之间间隔很大）。
若有 X11 显示，可去掉 `--dont-show-report` 以查看 PDF 报告界面。

**输出文件**（生成在 `extrinsics/` 下）：
- `68_66_64_65_67-camchain.yaml` —— **标定结果**：每个相机的内参 + `T_cn_cnm1`
  （4×4 矩阵，把点从 cam *n−1* 坐标系映射到 cam *n* 坐标系）。
- `68_66_64_65_67-results-cam.txt` —— 重投影误差 + 各基线。
- `68_66_64_65_67-report-cam.pdf` —— 诊断图。

**理想结果参考**（本套设备）：每个相机重投影误差 **±0.19–0.28 像素**；内侧基线
（66↔64、64↔65）约 **15 cm**，外侧基线（68↔66、65↔67）约 **27 cm**；各相机以 cam64
为中心呈 **±90° 扇形**分布。若某相机结果异常 → 回到第 2 步的 montage 检查其位姿，并重新
采集该对。

> 在 Linux 主机上，这些文件由 root 写入（Docker 以 root 运行）；可在 `docker run` 加
> `--user $(id -u):$(id -g)`，或事后 `chown`。（Docker Desktop 上无此问题。）

---

## 第 4 步 —— 导出 YAML

```bash
# (a) camchain -> 每个相机的 OpenCV 内参 + 可读的外参（4x4 + 四元数 + 欧拉角 + 基线）
python3 scripts/kalibr_to_opencv.py --camchain 68_66_64_65_67-camchain.yaml

# (b) 各相机相对某个参考相机的姿态角
python3 scripts/extrinsics_relative.py --camchain 68_66_64_65_67-camchain.yaml --ref 64

# (c) 内参 -> apollo-lite 视频拼接所用的 camchain_<id>.yaml（OpenCV FileStorage 格式）
python3 ../intrinsics/intrinsics_to_camchain.py
```

对接 apollo-lite 拼接模块时：相机间的变换写入 `params_hk/extrinsics_override.yaml`；
`camchain_<id>.yaml` 只携带内参，旋转留为单位阵/空。

**该用哪套内参？** Kalibr 估计的内参（来自 camchain）与外参自洽，但只基于重叠区域的视图
估计；棋盘格那套（`../intrinsics/`）则对全画幅覆盖更好。两者相差约 3% —— 决策前可运行
`scripts/compare_calib.py --camchain <文件>` 量化差异。

---

## 常见问题

| 现象 | 原因 / 解决 |
|---|---|
| Python AprilTag 检测器检测到 0 个 tag | 正常 —— 该标定板只有 Kalibr 自带检测器能识别。 |
| Kalibr 报 "0 observations"（无观测） | 标定板确实未被检测到 —— 检查 `--target` 与实际板一致；必要时用 `kalibr_create_target_pdf` 重新生成。 |
| 某个相机重投影误差偏大 | 该对的位姿太少 / 多样性不足 → 重新采集该对（变化距离与角度）。 |
| `merge_pairs.py` 时间戳冲突 | 增大某个 `--source` 的偏移量，使其超过单次采集的时长。 |
| 提取到的位姿过少 | 降低 `--min-still`（如 0.6）或增大 `--dup-thresh`；或录制更长的视频。 |

## 附录 —— 实时快照采集

除录制视频外，也可用 `scripts/capture_sync_rtsp.py` 同时从每个相机抓取一帧并打上相同
时间戳（每次抓取时保持标定板静止）：

```bash
python3 scripts/capture_sync_rtsp.py --out pairs/run01 --interval 0.5   # 或 --manual
```

其输出已经是第 3 步所需的文件夹结构；可跳过第 2 步直接进入第 3 步。

## 文件索引

- `scripts/extract_pairs_from_video.py` —— 视频 → 去重、共享时间戳的图像对。
- `scripts/merge_pairs.py` —— 把多个成对文件夹合并成一个多相机文件夹（时间偏移桥接）。
- `scripts/capture_sync_rtsp.py` —— 实时快照采集（替代方案）。
- `scripts/kalibr_to_opencv.py` —— camchain → OpenCV 内参 + 外参 YAML。
- `scripts/extrinsics_relative.py` —— 各相机相对参考相机的 yaw/roll/pitch。
- `scripts/compare_calib.py` —— Kalibr 内参 vs 棋盘格内参的对比。
- `../intrinsics/intrinsics_to_camchain.py` —— 内参 → apollo `camchain_<id>.yaml`。
- `aprilgrid_6x6.yaml` —— Kalibr 标定板定义。

录制视频、提取出的图像、bag 以及 Kalibr 输出都已被 **gitignore**（属于每套设备各自的
数据，运行本指南即可重新生成，不随仓库提交）。
