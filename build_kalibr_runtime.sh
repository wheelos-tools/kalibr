#!/usr/bin/env bash

set -euo pipefail

if [[ "$#" -ne 2 ]]; then
  echo "Usage: $0 KALIBR_SOURCE_DIR OUTPUT_ARCHIVE" >&2
  exit 2
fi

kalibr_source_dir="$1"
output_archive="$2"

if [[ ! -f "${kalibr_source_dir}/aslam_offline_calibration/kalibr/package.xml" ]]; then
  echo "Kalibr source directory is invalid: ${kalibr_source_dir}" >&2
  exit 2
fi

required_commands=(catkin_make_isolated cmake make tar /usr/bin/python3)
for required_command in "${required_commands[@]}"; do
  if ! command -v "${required_command}" >/dev/null 2>&1; then
    echo "Missing build command: ${required_command}" >&2
    echo "Install the camera-camera build dependencies documented in README.md." >&2
    exit 2
  fi
done

if [[ "${output_archive}" != /* ]]; then
  output_archive="$(pwd)/${output_archive}"
fi
mkdir -p "$(dirname "${output_archive}")"

kalibr_build_root="$(mktemp -d "${TMPDIR:-/tmp}/apollo-kalibr-camera-camera.XXXXXX")"
cleanup() {
  rm -rf "${kalibr_build_root}"
}
trap cleanup EXIT

# setup.py writes egg-info beside several source trees. Build from a private
# copy so the module-owned Bazel inputs stay immutable.
mkdir -p "${kalibr_build_root}/source" "${kalibr_build_root}/workspace"
cp -a "${kalibr_source_dir}/." "${kalibr_build_root}/source/"

export PYTHONNOUSERSITE=1
export PYTHONPATH=/usr/lib/python3/dist-packages

available_build_cpus="$(nproc)"
default_build_jobs=8
if (( available_build_cpus < default_build_jobs )); then
  default_build_jobs="${available_build_cpus}"
fi
kalibr_build_jobs="${KALIBR_BUILD_JOBS:-${default_build_jobs}}"
catkin_make_isolated \
  -C "${kalibr_build_root}/workspace" \
  --source "${kalibr_build_root}/source" \
  --build "${kalibr_build_root}/build" \
  --devel "${kalibr_build_root}/devel" \
  --install-space "${kalibr_build_root}/install" \
  --install \
  --only-pkg-with-deps kalibr \
  --make-args "-j${kalibr_build_jobs}" \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=/usr/bin/gcc \
    -DCMAKE_CXX_COMPILER=/usr/bin/g++ \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.10 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.10.so \
    -DBUILD_APRILTAGS_DEMO=OFF \
    -DBUILD_NUMPY_EIGEN_TEST_MODULE=OFF \
    -DBoost_NO_BOOST_CMAKE=ON \
    -DBOOST_ROOT=/usr \
    -DBoost_INCLUDE_DIR=/usr/include \
    -DBoost_LIBRARY_DIR_RELEASE=/usr/lib/x86_64-linux-gnu \
    -DCATKIN_ENABLE_TESTING=OFF

runtime_python_path="${kalibr_build_root}/install/lib/python3/dist-packages:/usr/lib/python3/dist-packages"
runtime_library_path="${kalibr_build_root}/install/lib"
env \
  PYTHONNOUSERSITE=1 \
  PYTHONPATH="${runtime_python_path}" \
  LD_LIBRARY_PATH="${runtime_library_path}" \
  MPLBACKEND=Agg \
  /usr/bin/python3 -c \
    'import numpy_eigen, sm, aslam_cv, aslam_cameras_april, aslam_backend, aslam_cv_backend, incremental_calibration, kalibr_common, kalibr_camera_calibration'

tar -C "${kalibr_build_root}/install" -czf "${output_archive}" .
echo "Created Apollo Kalibr runtime: ${output_archive}"
