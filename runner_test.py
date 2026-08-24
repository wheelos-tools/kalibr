#!/usr/bin/env python3

from pathlib import Path
import os
import tempfile
import unittest

import runner


class RunnerTest(unittest.TestCase):

  def test_builds_deterministic_two_camera_command(self):
    with tempfile.TemporaryDirectory() as temp_dir:
      root = Path(temp_dir)
      input_dir = root / "dataset"
      input_dir.mkdir()
      target = root / "target.yaml"
      target.write_text("target_type: aprilgrid\n", encoding="utf-8")
      args = runner.parse_args([
          "--input-dir", str(input_dir),
          "--target", str(target),
          "--kalibr-executable", "/bin/true",
      ])
      command = runner.build_command(args, "/bin/true")

    self.assertEqual("/bin/true", command[0])
    self.assertIn("--input-dir", command)
    self.assertIn("--topics", command)
    self.assertIn("/cam0/image_raw", command)
    self.assertIn("/cam1/image_raw", command)
    self.assertIn("--no-shuffle", command)
    self.assertIn("--dont-show-report", command)
    models = command[command.index("--models") + 1:command.index("--target")]
    self.assertEqual(["pinhole-radtan", "pinhole-radtan"], models)

  def test_main_dry_run(self):
    with tempfile.TemporaryDirectory() as temp_dir:
      root = Path(temp_dir)
      input_dir = root / "dataset"
      input_dir.mkdir()
      target = root / "target.yaml"
      target.write_text("target_type: aprilgrid\n", encoding="utf-8")
      result = runner.main([
          "--input-dir", str(input_dir),
          "--target", str(target),
          "--kalibr-executable", "/bin/true",
          "--dry-run",
      ])
    self.assertEqual(0, result)

  def test_packaged_runtime_environment_uses_system_python_packages(self):
    with tempfile.TemporaryDirectory() as temp_dir:
      runtime_root = Path(temp_dir)
      old_pythonhome = os.environ.get("PYTHONHOME")
      os.environ["PYTHONHOME"] = "/tmp/not-system-python"
      try:
        environment = runner.packaged_runtime_environment(runtime_root)
      finally:
        if old_pythonhome is None:
          os.environ.pop("PYTHONHOME", None)
        else:
          os.environ["PYTHONHOME"] = old_pythonhome

    self.assertNotIn("PYTHONHOME", environment)
    self.assertEqual("1", environment["PYTHONNOUSERSITE"])
    self.assertEqual("Agg", environment["MPLBACKEND"])
    self.assertTrue(environment["PYTHONPATH"].startswith(
        str(runtime_root / "lib" / "python3" / "dist-packages")))


if __name__ == "__main__":
  unittest.main()
