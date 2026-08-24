from __future__ import print_function

import os

import cv2
import numpy as np


class FolderImageDatasetValidationError(RuntimeError):
  """Raised when a two-camera PNG dataset violates the input contract."""

  def __init__(self, code, message, report=None):
    RuntimeError.__init__(self, message)
    self.code = code
    self.report = report or {}


class PairedImageDatasetManifest(object):
  """Validated file and timestamp index shared by the two camera readers."""

  def __init__(self, input_dir, timestamps_ns, files_by_camera, report):
    self.input_dir = input_dir
    self.timestamps_ns = timestamps_ns
    self.files_by_camera = files_by_camera
    self.report = report


def _validation_error(report, code, message, details=None):
  error = {
      "code": code,
      "message": message,
  }
  if details:
    error["details"] = details
  report["status"] = "FAILED"
  report.setdefault("errors", []).append(error)
  raise FolderImageDatasetValidationError(code, message, report)


def _scan_camera_directory(camera_dir, camera_name, report):
  files_by_stem = {}
  unsupported_images = []

  for entry in sorted(os.listdir(camera_dir)):
    path = os.path.join(camera_dir, entry)
    if not os.path.isfile(path):
      continue
    stem, extension = os.path.splitext(entry)
    if extension.lower() != ".png":
      if extension.lower() in (".bmp", ".jpg", ".jpeg", ".tif", ".tiff"):
        unsupported_images.append(entry)
      continue
    if not stem.isdigit():
      _validation_error(
          report,
          "INVALID_TIMESTAMP_FILENAME",
          "PNG filenames must be integer nanosecond timestamps.",
          {"camera": camera_name, "filename": entry})
    timestamp_ns = int(stem)
    if timestamp_ns < 0:
      _validation_error(
          report,
          "INVALID_TIMESTAMP_FILENAME",
          "PNG timestamps must be non-negative.",
          {"camera": camera_name, "filename": entry})
    if stem in files_by_stem:
      _validation_error(
          report,
          "DUPLICATE_TIMESTAMP",
          "A camera directory contains duplicate PNG timestamps.",
          {"camera": camera_name, "timestamp": stem})
    files_by_stem[stem] = path

  if unsupported_images:
    _validation_error(
        report,
        "UNSUPPORTED_IMAGE_FORMAT",
        "Only PNG images are accepted by the camera-camera calibrator.",
        {"camera": camera_name, "files": unsupported_images[:20]})
  if not files_by_stem:
    _validation_error(
        report,
        "NO_PNG_IMAGES",
        "No PNG images were found in {0}.".format(camera_name),
        {"camera": camera_name, "directory": camera_dir})
  return files_by_stem


def _validate_camera_images(camera_name, ordered_files, report):
  resolution = None
  for path in ordered_files:
    image = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if image is None:
      _validation_error(
          report,
          "UNREADABLE_PNG",
          "A PNG image could not be decoded.",
          {"camera": camera_name, "file": path})
    current_resolution = [int(image.shape[1]), int(image.shape[0])]
    if resolution is None:
      resolution = current_resolution
    elif current_resolution != resolution:
      _validation_error(
          report,
          "INCONSISTENT_RESOLUTION",
          "All images from one camera must have the same resolution.",
          {
              "camera": camera_name,
              "expected": resolution,
              "actual": current_resolution,
              "file": path,
          })
  return resolution


def inspectPairedImageDataset(input_dir, min_image_pairs=20):
  """Validate an input_dir/cam0 + cam1 PNG dataset and return its manifest."""
  input_dir = os.path.abspath(os.path.expanduser(input_dir))
  report = {
      "schemaVersion": 1,
      "status": "CHECKING",
      "inputDirectory": input_dir,
      "minimumImagePairs": int(min_image_pairs),
      "cameras": {},
      "errors": [],
  }

  if min_image_pairs < 1:
    _validation_error(
        report,
        "INVALID_MINIMUM_IMAGE_PAIRS",
        "The minimum image-pair count must be positive.")
  if not os.path.isdir(input_dir):
    _validation_error(
        report,
        "INPUT_DIRECTORY_NOT_FOUND",
        "The camera-camera input directory does not exist.",
        {"directory": input_dir})

  camera_names = ("cam0", "cam1")
  camera_dirs = {
      camera_name: os.path.join(input_dir, camera_name)
      for camera_name in camera_names
  }
  missing_cameras = [
      camera_name for camera_name, path in camera_dirs.items()
      if not os.path.isdir(path)
  ]
  if missing_cameras:
    _validation_error(
        report,
        "CAMERA_DIRECTORY_NOT_FOUND",
        "The input must contain cam0 and cam1 directories.",
        {"missing": missing_cameras})

  discovered_camera_dirs = sorted([
      entry for entry in os.listdir(input_dir)
      if entry.startswith("cam") and os.path.isdir(os.path.join(input_dir, entry))
  ])
  unexpected_cameras = [
      entry for entry in discovered_camera_dirs if entry not in camera_names
  ]
  if unexpected_cameras:
    _validation_error(
        report,
        "UNEXPECTED_CAMERA_DIRECTORY",
        "The first version accepts exactly two camera directories: cam0 and cam1.",
        {"unexpected": unexpected_cameras})

  files_by_camera_stem = {}
  for camera_name in camera_names:
    files_by_camera_stem[camera_name] = _scan_camera_directory(
        camera_dirs[camera_name], camera_name, report)

  cam0_stems = set(files_by_camera_stem["cam0"].keys())
  cam1_stems = set(files_by_camera_stem["cam1"].keys())
  only_cam0 = sorted(cam0_stems - cam1_stems, key=int)
  only_cam1 = sorted(cam1_stems - cam0_stems, key=int)
  if only_cam0 or only_cam1:
    _validation_error(
        report,
        "UNPAIRED_IMAGES",
        "cam0 and cam1 must contain exactly matching PNG timestamp filenames.",
        {
            "onlyCam0": only_cam0[:20],
            "onlyCam1": only_cam1[:20],
            "onlyCam0Count": len(only_cam0),
            "onlyCam1Count": len(only_cam1),
        })

  paired_stems = sorted(cam0_stems, key=int)
  pair_count = len(paired_stems)
  report["imagePairCount"] = pair_count
  if pair_count < min_image_pairs:
    _validation_error(
        report,
        "INSUFFICIENT_IMAGE_PAIRS",
        "The dataset does not contain enough paired PNG images.",
        {"actual": pair_count, "required": int(min_image_pairs)})

  timestamps_ns = [int(stem) for stem in paired_stems]
  if len(set(timestamps_ns)) != len(timestamps_ns):
    _validation_error(
        report,
        "DUPLICATE_NUMERIC_TIMESTAMP",
        "Different filenames resolve to the same integer timestamp.")

  files_by_camera = {}
  for camera_name in camera_names:
    ordered_files = [
        files_by_camera_stem[camera_name][stem] for stem in paired_stems
    ]
    resolution = _validate_camera_images(camera_name, ordered_files, report)
    files_by_camera[camera_name] = ordered_files
    report["cameras"][camera_name] = {
        "directory": camera_dirs[camera_name],
        "imageCount": len(ordered_files),
        "resolution": resolution,
    }

  report["firstTimestampNs"] = timestamps_ns[0]
  report["lastTimestampNs"] = timestamps_ns[-1]
  report["status"] = "INPUT_VALIDATED"
  return PairedImageDatasetManifest(
      input_dir, timestamps_ns, files_by_camera, report)


class FolderImageDatasetReaderIterator(object):
  def __init__(self, dataset, indices=None):
    self.dataset = dataset
    if indices is None:
      self.indices = np.arange(dataset.numImages())
    else:
      self.indices = indices
    self.iter = self.indices.__iter__()

  def __iter__(self):
    return self

  def next(self):
    return self.dataset.getImage(next(self.iter))

  def __next__(self):
    return self.dataset.getImage(next(self.iter))


class FolderImageDatasetReader(object):
  """Kalibr image dataset backed by one camera in a paired PNG manifest."""

  def __init__(self, manifest, camera_name, topic):
    if camera_name not in ("cam0", "cam1"):
      raise ValueError("camera_name must be cam0 or cam1")
    self.manifest = manifest
    self.camera_name = camera_name
    self.topic = topic
    self.files = manifest.files_by_camera[camera_name]
    self.timestamps_ns = manifest.timestamps_ns
    self.indices = np.arange(len(self.files))

  def __iter__(self):
    return self.readDataset()

  def readDataset(self):
    return FolderImageDatasetReaderIterator(self, self.indices)

  def readDatasetShuffle(self):
    indices = np.array(self.indices, copy=True)
    np.random.shuffle(indices)
    return FolderImageDatasetReaderIterator(self, indices)

  def numImages(self):
    return len(self.files)

  def getImage(self, idx):
    import aslam_cv as acv

    idx = int(idx)
    image = cv2.imread(self.files[idx], cv2.IMREAD_GRAYSCALE)
    if image is None:
      raise RuntimeError("Could not decode PNG image: {0}".format(self.files[idx]))
    timestamp_ns = int(self.timestamps_ns[idx])
    seconds, nanoseconds = divmod(timestamp_ns, 1000000000)
    return (acv.Time(seconds, nanoseconds), image)
