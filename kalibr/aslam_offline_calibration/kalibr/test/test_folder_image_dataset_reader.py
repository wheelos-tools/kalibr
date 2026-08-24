#!/usr/bin/env python3

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest

import cv2
import numpy as np


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "python"
    / "kalibr_common"
    / "FolderImageDatasetReader.py"
)
SPEC = importlib.util.spec_from_file_location(
    "folder_image_dataset_reader", str(MODULE_PATH))
folder_reader = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(folder_reader)


class FakeTime(object):
  def __init__(self, seconds, nanoseconds):
    self.seconds = seconds
    self.nanoseconds = nanoseconds


class FolderImageDatasetReaderTest(unittest.TestCase):

  def setUp(self):
    self.temp_dir = tempfile.TemporaryDirectory()
    self.input_dir = Path(self.temp_dir.name)
    (self.input_dir / "cam0").mkdir()
    (self.input_dir / "cam1").mkdir()

  def tearDown(self):
    self.temp_dir.cleanup()

  def write_pair(self, timestamp_ns, cam0_shape=(12, 16), cam1_shape=(10, 14)):
    stem = str(timestamp_ns)
    cam0 = np.full(cam0_shape, timestamp_ns % 251, dtype=np.uint8)
    cam1 = np.full(cam1_shape, (timestamp_ns + 1) % 251, dtype=np.uint8)
    self.assertTrue(cv2.imwrite(str(self.input_dir / "cam0" / (stem + ".png")), cam0))
    self.assertTrue(cv2.imwrite(str(self.input_dir / "cam1" / (stem + ".png")), cam1))

  def assert_validation_error(self, code, callback):
    with self.assertRaises(folder_reader.FolderImageDatasetValidationError) as context:
      callback()
    self.assertEqual(code, context.exception.code)
    self.assertEqual("FAILED", context.exception.report["status"])
    self.assertEqual(code, context.exception.report["errors"][-1]["code"])

  def test_valid_pair_manifest_and_reader(self):
    timestamps = [1000000000000000000, 1000000001000000000, 1000000002000000000]
    for timestamp_ns in reversed(timestamps):
      self.write_pair(timestamp_ns)

    manifest = folder_reader.inspectPairedImageDataset(
        str(self.input_dir), min_image_pairs=3)
    self.assertEqual(timestamps, manifest.timestamps_ns)
    self.assertEqual(3, manifest.report["imagePairCount"])
    self.assertEqual([16, 12], manifest.report["cameras"]["cam0"]["resolution"])
    self.assertEqual([14, 10], manifest.report["cameras"]["cam1"]["resolution"])
    self.assertEqual("INPUT_VALIDATED", manifest.report["status"])

    previous_aslam_cv = sys.modules.get("aslam_cv")
    sys.modules["aslam_cv"] = types.SimpleNamespace(Time=FakeTime)
    try:
      reader = folder_reader.FolderImageDatasetReader(
          manifest, "cam0", "/cam0/image_raw")
      timestamp, image = reader.getImage(1)
    finally:
      if previous_aslam_cv is None:
        del sys.modules["aslam_cv"]
      else:
        sys.modules["aslam_cv"] = previous_aslam_cv

    self.assertEqual(3, reader.numImages())
    self.assertEqual(1000000001, timestamp.seconds)
    self.assertEqual(0, timestamp.nanoseconds)
    self.assertEqual((12, 16), image.shape)
    self.assertEqual(np.uint8, image.dtype)

  def test_rejects_insufficient_pairs(self):
    self.write_pair(1000000000000000000)
    self.assert_validation_error(
        "INSUFFICIENT_IMAGE_PAIRS",
        lambda: folder_reader.inspectPairedImageDataset(
            str(self.input_dir), min_image_pairs=2))

  def test_rejects_unpaired_images(self):
    self.write_pair(1000000000000000000)
    extra = np.zeros((12, 16), dtype=np.uint8)
    cv2.imwrite(
        str(self.input_dir / "cam0" / "1000000001000000000.png"), extra)
    self.assert_validation_error(
        "UNPAIRED_IMAGES",
        lambda: folder_reader.inspectPairedImageDataset(
            str(self.input_dir), min_image_pairs=1))

  def test_rejects_non_timestamp_filename(self):
    image = np.zeros((12, 16), dtype=np.uint8)
    cv2.imwrite(str(self.input_dir / "cam0" / "frame.png"), image)
    cv2.imwrite(str(self.input_dir / "cam1" / "frame.png"), image)
    self.assert_validation_error(
        "INVALID_TIMESTAMP_FILENAME",
        lambda: folder_reader.inspectPairedImageDataset(
            str(self.input_dir), min_image_pairs=1))

  def test_rejects_inconsistent_resolution(self):
    self.write_pair(1000000000000000000)
    self.write_pair(1000000001000000000, cam0_shape=(13, 16))
    self.assert_validation_error(
        "INCONSISTENT_RESOLUTION",
        lambda: folder_reader.inspectPairedImageDataset(
            str(self.input_dir), min_image_pairs=2))

  def test_rejects_non_png_images(self):
    self.write_pair(1000000000000000000)
    jpeg = np.zeros((12, 16), dtype=np.uint8)
    cv2.imwrite(str(self.input_dir / "cam0" / "ignored.jpg"), jpeg)
    self.assert_validation_error(
        "UNSUPPORTED_IMAGE_FORMAT",
        lambda: folder_reader.inspectPairedImageDataset(
            str(self.input_dir), min_image_pairs=1))


if __name__ == "__main__":
  unittest.main()
