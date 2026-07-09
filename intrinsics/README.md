# Best-of intrinsics for camera2camera extrinsic calibration

Five Hikvision IP cameras (`192.168.1.64`–`192.168.1.68`), same lens model,
each at zoom step 14/40 (~5.2 mm). All passed quality gates.
Format matches `whl-cal/camera2camera`'s `intrinsics_path` expectation (no
conversion needed).

| File | Source run | fx | k1 | per-view RMS p95 |
|---|---|---:|---:|---:|
| `intrinsics_cam64.yaml` | round03 | 1377.1 | −0.418 | 0.21 px |
| `intrinsics_cam65.yaml` | round12 | 1378.0 | −0.426 | 0.35 px |
| `intrinsics_cam66.yaml` | round10 | 1377.0 | −0.424 | 0.30 px |
| `intrinsics_cam67.yaml` | round09 | 1380.7 | −0.420 | 0.30 px |
| `intrinsics_cam68.yaml` | round08 | 1376.7 | −0.421 | 0.27 px |

Cross-camera consistency: `fx` spread 1376.7–1380.7 (0.3%), `k1` spread
−0.418 to −0.426 (2%). Identical lens family across all five units.

All carry the same metadata downstream (`distortion_model: plumb_bob`,
`image_width: 1920`, `image_height: 1080`).

## Caveats per camera (informational, not blocking)

All five share these soft warnings (corner coverage was the operator's
biggest weakness; usable for c2c, may need re-shoot if you ever want
production-grade undistortion to full frame):

- `valid_roi` 34–62 % of frame after `getOptimalNewCameraMatrix(alpha=1)`
- `radial monotonicity` ≈ 0.33–0.37 (above zero, below the 0.5 production
  target)
- grid_counts center-heavy; corner cells under-sampled

cam65 carries the highest per-view RMS (0.35 px on round12; cam64–cam68
sit at 0.21–0.30 px). Still well clear of the 1.0 px hard threshold and
the K/D values match the family within 0.3 %.

## Using these in camera2camera

Per the `camera2camera_quickstart.md` schema:

```yaml
cameras:
  parent:
    frame_id: camera_64                # set to your TF frame
    image_directory: run01/parent       # paired images, same filename stems
    intrinsics_path: intrinsics/intrinsics_cam64.yaml
  child:
    frame_id: camera_65
    image_directory: run01/child
    intrinsics_path: intrinsics/intrinsics_cam65.yaml
target:
  type: checkerboard
  pattern_size: [11, 8]
  square_size_m: 0.045
extraction:
  min_bbox_area_ratio: 0.003
  min_edge_margin_px: 16.0
  max_pnp_reprojection_rms_px: 1.5
optimization:
  min_pairs: 8
  loss: huber
  f_scale: 1.0
  max_nfev: 300
metrics:
  warning_final_rms_px: 1.0
  warning_pair_rms_p95_px: 1.5
  warning_holdout_rms_px: 1.5
  warning_epipolar_p95_px: 1.0
output:
  directory: outputs/camera2camera/64_65
```

A starter template is staged at `camera2camera_config_template.yaml` in
this directory — copy it per pair and edit `frame_id`, `image_directory`,
and `output.directory`.

## How the files were chosen

Per-camera best run (full report at `../cams/report.md`):

- cam64 → `round03_chessboard_64` (only round; passed first time)
- cam65 → `round12_chessboard` (two recaptures; round12 chosen — lower p95
  RMS than round11, identical K/D)
- cam66 → `round10_chessboard` (replaced round05 which had `fx=2951`,
  `k3=−7.2` — overfit failure)
- cam67 → `round09_chessboard` (replaced round06 which had `fx=2112`,
  `rmono=−4.9` — overfit failure)
- cam68 → `round08_chessboard` (replaced round07 which had `fx=1720`,
  `rmono=−0.27` — non-monotonic distortion failure)

The failed rounds were all `N=9` (the minimum) with the board held too
far from the camera (`bbox_mean ~0.03`). The passing rounds used the
`N=20` protocol with the board filling 8–22 % of the frame.

## Source files (full provenance)

Full per-run artifacts (diagnostics, comparison views, per-view CSVs)
remain under `../cams/<run_dir>/`. The files staged here are verbatim
copies of each run's `calibration.yaml`; nothing was edited.
