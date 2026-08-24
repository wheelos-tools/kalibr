# Camera-Camera 标定（PNG 第一版）

`apollo-camera-camera` 分支保存 Apollo 版本的 Kalibr camera-camera 标定模块。
该分支根目录可以完整复制到 Apollo 源码树中独立构建和运行，第一版只接收两组配对的
普通 PNG 作为输入。

运行所需的 Kalibr/Aslam 源码、构建兼容修改、PNG reader、默认 AprilGrid 配置均位于
本目录的 `kalibr/` 下，不依赖目标仓库中的 `data/kalibr` 或外部 `/catkin_ws`。

## 复制到 Apollo

检出 `data/kalibr` 仓库的发布分支：

```bash
git clone --branch apollo-camera-camera --single-branch \
  git@github.com:wheelos-tools/kalibr.git <KALIBR_APOLLO_CHECKOUT>
```

假设目标 Apollo 根目录是 `<APOLLO_ROOT>`，将该分支的所有已提交内容复制到目标模块：

```bash
mkdir -p <APOLLO_ROOT>/modules/calibration/camera_camera_calibrator
git -C <KALIBR_APOLLO_CHECKOUT> archive HEAD | \
  tar -x -C <APOLLO_ROOT>/modules/calibration/camera_camera_calibrator
```

上述方式不会复制 Kalibr 仓库的 `.git` 元数据。复制后目录至少应包含：

```text
modules/calibration/camera_camera_calibrator/
├── BUILD
├── README.md
├── build_kalibr_runtime.sh
├── install_deps.sh
├── kalibr/
├── runner.py
└── runner_test.py
```

不需要修改 `data/kalibr`。`bash apollo.sh build_opt_gpu calibration` 会递归构建
`//modules/calibration/...`，因此能够发现本目录中的 BUILD 目标。

## 实现边界

Python 只负责输入契约、运行时加载和命令入口；数值计算仍由原始 Kalibr/Aslam C++
实现：

```text
Apollo py_binary
  -> 解包同一 Bazel 目标生成的 kalibr_runtime.tar.gz
  -> /usr/bin/python3 + Boost.Python bindings
  -> AprilGrid 检测、相机模型、图初始化、Aslam 非线性优化
```

`kalibr_runtime.tar.gz` 不是外部 `/catkin_ws` 产物。Bazel 的
`kalibr_runtime_archive` 目标会在 Apollo Ubuntu 22.04 开发环境内调用原始
Catkin/CMake，编译 36 个依赖包及其 C++ 共享库，再执行完整 binding 导入检查。
标定残差、相机模型、图构建、优化参数和报告计算均未重写。

为适配 Apollo 的 Python 3.10/Boost 1.74 install-space，只修改了构建和加载兼容层：

- Boost.Python 库名从固定 `python38` 改为按 Python ABI 推导；
- 修复新 Boost 中已删除的字节序头文件和 binding 安装目录；
- ROS bag import 在 PNG 模式下变为可选；
- 关闭无关的 V4L2 AprilTag demo 和 `numpy_eigen` 测试 binding；
- 默认使用无 GUI 后端生成 PDF。

这些改动不进入标定数值路径。

## 输入约定

```text
dataset/
├── cam0/
│   ├── 1000000000000000000.png
│   └── 1000000001000000000.png
└── cam1/
    ├── 1000000000000000000.png
    └── 1000000001000000000.png
```

`cam0` 是父相机，`cam1` 是子相机。文件名必须是非负整数纳秒时间戳，两目录必须
包含完全相同的文件名；只接受 `.png`，每个相机内部的分辨率必须固定。

默认门槛为：

- 至少 20 对可解码 PNG；
- 每个相机至少 15 张能检测到 AprilGrid；
- 至少 15 对同步视图含有两相机共同可见的 AprilGrid 角点。

任一条件不满足时以退出码 2 结束，在 `<output-prefix>-input-check.json` 中记录错误码、
实际数量和要求数量，不进入优化。检测到标定板但内参初始化所需的姿态/距离变化不足时，
也会以 `INSUFFICIENT_CAM0_POSE_DIVERSITY` 或
`INSUFFICIENT_CAM1_POSE_DIVERSITY` 明确提示。

## 安装依赖

进入 Apollo Ubuntu 22.04 开发容器，在 Apollo 根目录执行：

```bash
sudo bash modules/calibration/camera_camera_calibrator/install_deps.sh
```

该脚本安装 Catkin、系统 Python 3.10 开发包、Boost、OpenCV、Eigen、SuiteSparse、TBB
以及报告生成所需的 Python 包。构建和运行均固定使用 `/usr/bin/python3` 与系统库，避免
Apollo 工作区中其他 Conda/pip NumPy 与系统 OpenCV 发生 ABI 冲突。

## 编译

依赖安装完成后，在 Apollo 根目录执行：

```bash
bash apollo.sh build_opt_gpu calibration
```

也可以只构建该入口：

```bash
bazel build --config=opt --config=gpu \
  //modules/calibration/camera_camera_calibrator:camera_camera_calibrator
```

冷构建会编译原始 C++/bindings，耗时明显长于普通 `py_binary`；输入源码未变化时会命中
Bazel 缓存。成功后应同时存在：

```text
bazel-bin/modules/calibration/camera_camera_calibrator/camera_camera_calibrator
bazel-bin/modules/calibration/camera_camera_calibrator/kalibr_runtime.tar.gz
```

无需 source ROS/Kalibr 工作空间，也无需设置 `/catkin_ws`。环境变量
`KALIBR_CALIBRATE_CAMERAS` 和参数 `--kalibr-executable` 仅保留给迁移前后 A/B 对比；
未指定时优先使用 Apollo 构建产物。

## 运行

```bash
bazel-bin/modules/calibration/camera_camera_calibrator/camera_camera_calibrator \
  --input-dir /apollo/data/calibration/camera_camera/dataset \
  --output-prefix /apollo/data/calibration/camera_camera/result
```

默认会从二进制 runfiles 中读取本模块的
`kalibr/extrinsics/aprilgrid_6x6.yaml`。如果标定板尺寸不同，必须通过 `--target` 指定
匹配实物的 YAML，例如：

```bash
--target /apollo/modules/calibration/camera_camera_calibrator/kalibr/extrinsics/aprilgrid_6x6.yaml
```

默认两个相机均使用 `pinhole-radtan`，固定标签为 `/cam0/image_raw` 和
`/cam1/image_raw`。为便于迁移前后复现，默认关闭 observation shuffle，不弹出 GUI；
PDF 报告仍会生成。可通过 `--cam0-model`、`--cam1-model`、`--target`、
`--min-image-pairs` 和 `--min-mutual-observations` 覆盖配置。

默认输出前缀是 `INPUT_DIR/camera_camera`：

```text
camera_camera-input-check.json
camera_camera-camchain.yaml
camera_camera-results-cam.txt
camera_camera-report-cam.pdf
```

## 迁移一致性验证

用同一批 PNG 分别执行历史路径
`kalibr_bagcreater -> kalibr_calibrate_cameras --bag` 和新入口。历史路径同样加
`--no-shuffle --dont-show-report`，并保证 models、target、topics 和
`--approx-sync` 一致，然后比较 camchain、重投影误差、内参及 `T_cn_cnm1`。当前仓库
没有随附真实双相机 AprilGrid 数据，因此端到端数值一致性需要用项目实采数据完成。
