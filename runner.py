#!/usr/bin/env python3

"""Apollo command entry for Kalibr camera-camera calibration from PNG pairs."""

import argparse
from contextlib import contextmanager
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile


DEFAULT_KALIBR_EXECUTABLES = (
    "/catkin_ws/devel/lib/kalibr/kalibr_calibrate_cameras",
)
PACKAGED_RUNTIME = (
    "modules/calibration/camera_camera_calibrator/kalibr_runtime.tar.gz")
PACKAGED_TARGET = (
    "modules/calibration/camera_camera_calibrator/kalibr/extrinsics/"
    "aprilgrid_6x6.yaml")
SYSTEM_PYTHON = "/usr/bin/python3"


def _executable_path(value):
  if not value:
    return ""
  if os.path.sep not in value:
    return shutil.which(value) or ""
  path = Path(value).expanduser().resolve()
  return str(path) if path.is_file() and os.access(str(path), os.X_OK) else ""


def resolve_external_kalibr_executable(explicit=""):
  candidates = [
      explicit,
      os.environ.get("KALIBR_CALIBRATE_CAMERAS", ""),
  ]
  for candidate in candidates:
    resolved = _executable_path(candidate)
    if resolved:
      return resolved
  return ""


def _runfile_path(relative_path):
  candidates = []
  for env_name in ("RUNFILES_DIR", "TEST_SRCDIR"):
    runfiles_dir = os.environ.get(env_name, "")
    if runfiles_dir:
      candidates.extend([
          Path(runfiles_dir) / "apollo" / relative_path,
          Path(runfiles_dir) / relative_path,
      ])

  executable_paths = [
      Path(os.path.abspath(sys.argv[0])),
      Path(sys.argv[0]).resolve(),
  ]
  for executable_path in executable_paths:
    executable_runfiles = Path(str(executable_path) + ".runfiles")
    candidates.append(executable_runfiles / "apollo" / relative_path)

  source_files = [
      Path(os.path.abspath(__file__)),
      Path(__file__).resolve(),
  ]
  for source_file in source_files:
    candidates.append(source_file.parent / Path(relative_path).name)
    for parent in source_file.parents:
      candidates.append(parent / relative_path)

  for candidate in candidates:
    if candidate.is_file():
      return candidate
  return None


def packaged_runtime_archive():
  return _runfile_path(PACKAGED_RUNTIME)


def _prepend_path(path, existing=""):
  return str(path) if not existing else str(path) + os.pathsep + existing


def packaged_runtime_environment(runtime_root, show_report=False):
  environment = os.environ.copy()
  environment.pop("PYTHONHOME", None)
  environment["PYTHONNOUSERSITE"] = "1"
  environment["PYTHONPATH"] = os.pathsep.join([
      str(runtime_root / "lib" / "python3" / "dist-packages"),
      "/usr/lib/python3/dist-packages",
  ])
  environment["LD_LIBRARY_PATH"] = _prepend_path(
      runtime_root / "lib", environment.get("LD_LIBRARY_PATH", ""))
  if not show_report:
    environment["MPLBACKEND"] = "Agg"
  return environment


@contextmanager
def resolve_kalibr_runtime(explicit="", show_report=False):
  external_executable = resolve_external_kalibr_executable(explicit)
  if external_executable:
    yield [external_executable], None
    return

  runtime_archive = packaged_runtime_archive()
  if runtime_archive is None:
    for legacy_candidate in (
        "kalibr_calibrate_cameras",
    ) + DEFAULT_KALIBR_EXECUTABLES:
      legacy_executable = _executable_path(legacy_candidate)
      if legacy_executable:
        yield [legacy_executable], None
        return
    raise RuntimeError(
        "The Apollo-built Kalibr runtime was not found. Build target "
        "//modules/calibration/camera_camera_calibrator:camera_camera_calibrator "
        "or set KALIBR_CALIBRATE_CAMERAS explicitly.")

  with tempfile.TemporaryDirectory(prefix="apollo-kalibr-runtime-") as temp_dir:
    runtime_root = Path(temp_dir)
    with tarfile.open(str(runtime_archive), "r:gz") as archive:
      archive.extractall(str(runtime_root))
    executable = runtime_root / "lib" / "kalibr" / "kalibr_calibrate_cameras"
    if not executable.is_file():
      raise RuntimeError(
          "The packaged Kalibr runtime is incomplete: {0}".format(executable))
    if not Path(SYSTEM_PYTHON).is_file():
      raise RuntimeError("System Python is required at {0}".format(SYSTEM_PYTHON))
    yield (
        [SYSTEM_PYTHON, str(executable)],
        packaged_runtime_environment(runtime_root, show_report),
    )


def default_target_path():
  apollo_root = Path(os.environ.get("APOLLO_ROOT_DIR", "/apollo"))
  source_target = apollo_root / PACKAGED_TARGET
  if source_target.is_file():
    return str(source_target)
  runfile_target = _runfile_path(PACKAGED_TARGET)
  return str(runfile_target) if runfile_target else str(source_target)


def parse_args(argv=None):
  parser = argparse.ArgumentParser(
      description=(
          "Run the original Kalibr camera-camera solver directly on paired "
          "PNG directories input_dir/cam0 and input_dir/cam1."))
  parser.add_argument("--input-dir", required=True,
                      help="Dataset directory containing cam0/ and cam1/")
  parser.add_argument("--output-prefix", default="",
                      help="Output path prefix; defaults to INPUT_DIR/camera_camera")
  parser.add_argument("--target", default=default_target_path(),
                      help="Kalibr target YAML")
  parser.add_argument("--kalibr-executable", default="",
                      help="Explicit kalibr_calibrate_cameras executable")
  parser.add_argument("--cam0-model", default="pinhole-radtan")
  parser.add_argument("--cam1-model", default="pinhole-radtan")
  parser.add_argument("--min-image-pairs", type=int, default=20)
  parser.add_argument("--min-mutual-observations", type=int, default=15)
  parser.add_argument("--approx-sync", type=float, default=0.01)
  parser.add_argument("--show-report", action="store_true",
                      help="Open the generated report after calibration")
  parser.add_argument("--shuffle", action="store_true",
                      help="Allow Kalibr to shuffle observations; disabled by default for parity tests")
  parser.add_argument("--dry-run", action="store_true",
                      help="Print the Kalibr command without running it")
  return parser.parse_args(argv)


def build_command(args, kalibr_executable):
  input_dir = str(Path(args.input_dir).expanduser().resolve())
  output_prefix = (
      str(Path(args.output_prefix).expanduser().resolve())
      if args.output_prefix
      else str(Path(input_dir) / "camera_camera")
  )
  target = str(Path(args.target).expanduser().resolve())
  command = (
      list(kalibr_executable)
      if isinstance(kalibr_executable, (list, tuple))
      else [kalibr_executable]
  ) + [
      "--input-dir", input_dir,
      "--topics", "/cam0/image_raw", "/cam1/image_raw",
      "--models", args.cam0_model, args.cam1_model,
      "--target", target,
      "--approx-sync", str(args.approx_sync),
      "--min-image-pairs", str(args.min_image_pairs),
      "--min-mutual-observations", str(args.min_mutual_observations),
      "--output-prefix", output_prefix,
  ]
  if not args.show_report:
    command.append("--dont-show-report")
  if not args.shuffle:
    command.append("--no-shuffle")
  return command


def main(argv=None):
  args = parse_args(argv)
  if args.min_image_pairs < 1 or args.min_mutual_observations < 1:
    print("minimum image and observation counts must be positive", file=sys.stderr)
    return 2
  input_dir = Path(args.input_dir).expanduser().resolve()
  if not input_dir.is_dir():
    print("input directory does not exist: {0}".format(input_dir), file=sys.stderr)
    return 2
  target = Path(args.target).expanduser().resolve()
  if not target.is_file():
    print("target YAML does not exist: {0}".format(target), file=sys.stderr)
    return 2
  try:
    with resolve_kalibr_runtime(
        args.kalibr_executable, args.show_report) as runtime:
      kalibr_command, runtime_environment = runtime
      command = build_command(args, kalibr_command)
      print("Running camera-camera calibration:")
      print("  " + " ".join(shlex.quote(item) for item in command))
      if args.dry_run:
        return 0
      return subprocess.call(command, env=runtime_environment)
  except (OSError, RuntimeError, tarfile.TarError) as error:
    print("failed to start Kalibr: {0}".format(error), file=sys.stderr)
    return 2


if __name__ == "__main__":
  sys.exit(main())
