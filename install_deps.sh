#!/usr/bin/env bash

set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run this script as root inside the Apollo development container:" >&2
  echo "  sudo bash modules/calibration/camera_camera_calibrator/install_deps.sh" >&2
  exit 2
fi

export DEBIAN_FRONTEND=noninteractive

apt-get update
apt-get install -y --no-install-recommends \
  build-essential \
  catkin \
  cmake \
  libblas-dev \
  libboost-all-dev \
  libeigen3-dev \
  liblapack-dev \
  libopencv-dev \
  libsuitesparse-dev \
  libtbb-dev \
  python3-catkin \
  python3-catkin-pkg \
  python3-dev \
  python3-empy \
  python3-igraph \
  python3-matplotlib \
  python3-numpy \
  python3-opencv \
  python3-pil \
  python3-setuptools \
  python3-yaml

echo "Camera-camera Kalibr dependencies installed successfully."
