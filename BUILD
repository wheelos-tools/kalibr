load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")
load("//tools/install:install.bzl", "install", "install_src_files")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "kalibr_sources",
    srcs = glob(
        ["kalibr/**"],
        exclude = [
            "kalibr/**/__pycache__/**",
            "kalibr/**/*.egg-info/**",
            "kalibr/**/*.pyc",
        ],
    ),
)

genrule(
    name = "kalibr_runtime_archive",
    srcs = [
        "build_kalibr_runtime.sh",
        ":kalibr_sources",
    ],
    outs = ["kalibr_runtime.tar.gz"],
    cmd = "bash $(location build_kalibr_runtime.sh) " +
          "$$(pwd)/modules/calibration/camera_camera_calibrator/kalibr $@",
    local = 1,
    tags = ["no-remote"],
)

py_library(
    name = "runner_lib",
    srcs = ["runner.py"],
)

py_binary(
    name = "camera_camera_calibrator",
    srcs = ["runner.py"],
    data = [
        ":kalibr_runtime_archive",
        "kalibr/extrinsics/aprilgrid_6x6.yaml",
    ],
    main = "runner.py",
)

py_test(
    name = "runner_test",
    srcs = ["runner_test.py"],
    deps = [":runner_lib"],
)

install(
    name = "install",
    data_dest = "calibration/camera_camera_calibrator",
    targets = [":camera_camera_calibrator"],
)

install_src_files(
    name = "install_src",
    dest = "calibration/camera_camera_calibrator/src",
    filter = "*",
    src_dir = ["."],
)
