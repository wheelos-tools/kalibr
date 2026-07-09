# Camera-to-Camera Calibration — Detailed Guide

A complete, reproducible walkthrough for calibrating the relative pose
(**extrinsics**) between two overlapping RTSP cameras with Kalibr + an AprilGrid,
covering the full lifecycle:

1. [Install & set up the environment](#1-install--set-up-the-environment)
2. [Capture the data](#2-capture-the-data)
3. [Convert the data into calibration format](#3-convert-the-data-into-calibration-format)
4. [Calibrate](#4-calibrate)
5. [Evaluate the result](#5-evaluate-the-result)

This is the **pairwise** (two-camera) procedure — the fundamental unit. To chain
several pairs into one N-camera rig (e.g. `68→66→64→65→67`), calibrate each pair
here, then follow [`EXTRINSIC_CALIBRATION_GUIDE.md`](EXTRINSIC_CALIBRATION_GUIDE.md)
(它的中文版：[`EXTRINSIC_CALIBRATION_GUIDE_zh.md`](EXTRINSIC_CALIBRATION_GUIDE_zh.md))
for the joint solve.

All commands run **from the `extrinsics/` directory**; scripts are in `scripts/`,
the target is `aprilgrid_6x6.yaml`.

### Concepts (read once)

- **What we solve.** The 4×4 rigid transform `T_cn_cnm1` mapping a 3-D point from
  the parent camera's frame into the child camera's frame — i.e. the child's
  position + orientation relative to the parent. Kalibr also (re-)estimates each
  camera's intrinsics as part of the same optimization.
- **Why AprilGrid, not a checkerboard.** Every tag has a unique ID, so a
  **partial** view of the board still contributes. That is what makes ~30 % FOV
  overlap between neighbours workable — a checkerboard must be seen whole.
- **Why no hardware sync is needed.** The cameras are independent RTSP streams
  (50–150 ms jitter). Extrinsics are a **static** transform, so we hold the board
  still per pose and give a pose's two frames **one identical synthetic
  timestamp**; Kalibr fuses them at `--approx-sync 0.01`. (See Steps 2–3.)
- **⚠️ Detector caveat.** Generic Python AprilTag detectors (`pupil-apriltags`,
  OpenCV `aruco`) **fail** to decode this board; **Kalibr's own detector
  succeeds** (results are sub-pixel). Verify detection by running Kalibr, not a
  Python probe.

---

## 1. Install & set up the environment

Two toolchains are involved:

- **Kalibr** — runs inside a **Docker** container (ROS Noetic). Used in Steps 4.
- **Python 3** helper scripts (extract / convert / evaluate) — run on the **host**.

### 1a. Kalibr (Docker)

The repo ships a Dockerfile (`Dockerfile_ros1_20_04`, ROS Noetic + a catkin build
of Kalibr). Build the image once, from the repo root:

```bash
cd /path/to/kalibr            # repo root (parent of extrinsics/)
docker build -t kalibr -f Dockerfile_ros1_20_04 .
```

This takes 15–40 min the first time (it compiles Kalibr). Verify:

```bash
docker images | grep kalibr           # -> kalibr  latest  ...
docker run --rm --entrypoint bash kalibr -c \
  'source /catkin_ws/devel/setup.bash && rosrun kalibr kalibr_calibrate_cameras --help | head -3'
```

Notes:
- The image's default `ENTRYPOINT` sets `KALIBR_MANUAL_FOCAL_LENGTH_INIT=1`. We
  override the entrypoint with `bash -c` (Step 4) and don't rely on it — AprilGrid
  gives a good focal init automatically. If a run ever fails to initialise the
  focal length, add `-e KALIBR_MANUAL_FOCAL_LENGTH_INIT=1` to the `docker run`.
- On a headless host, always pass `--dont-show-report` (Step 4). To see the GUI
  report, forward X11 (`-e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw`)
  and drop that flag.
- Files written from the container are `root`-owned on a Linux host — add
  `--user $(id -u):$(id -g)` to `docker run`, or `chown` afterwards.

### 1b. Python helpers (host)

Python 3.8+ with three packages:

```bash
python3 -m pip install --user "numpy>=1.21" opencv-python pyyaml
python3 -c "import cv2, numpy, yaml; print('cv2', cv2.__version__, '| numpy', numpy.__version__)"
```

> **Known gotcha.** A corrupted user-site `numpy` can import but crash inside
> `numpy.ma` (e.g. `np.median` → `ImportError: cannot import name 'core'`). Fix
> with `python3 -m pip install --user --force-reinstall --no-cache-dir numpy`.

### 1c. Prerequisites checklist

- [ ] `kalibr` Docker image built (1a).
- [ ] Host Python + `opencv-python`, `numpy`, `pyyaml` (1b).
- [ ] Physical **AprilGrid** printed (`april_6x6_80x80cm_A0.pdf`); its geometry
      matches `aprilgrid_6x6.yaml` (6×6 tags, tagSize 0.088 m, tagSpacing 0.3).
- [ ] *(optional)* Per-camera chessboard intrinsics in `../intrinsics/intrinsics_cam*.yaml`
      — only needed for the cross-check in [Step 5](#5-evaluate-the-result).

---

## 2. Capture the data

Goal: for the pair, record footage where the AprilGrid is visible in **both**
cameras across many well-separated poses.

### Operator protocol

- Keep the board in the pair's **overlap region** (visible to both cameras at once).
- **Stop-and-go:** move → **hold still 3–5 s** → move. The stillness is what lets
  us ignore the missing hardware sync.
- **~20–30 distinct poses**, spanning:
  - **distance** — near / mid / far (board filling roughly 10–40 % of frame),
  - **tilt** — pan and tilt the board out of the fronto-parallel plane,
  - **position** — sweep across the overlap strip (left/centre/right, high/low).
- Avoid motion blur; keep the board large enough that tags are crisp.
- Poses that are only fronto-parallel or all at one distance leave the intrinsics
  poorly constrained — **variety is what makes the solve well-posed.**

### Two capture methods

**A. Record a video per camera (recommended).** Start both cameras of the pair at
the same moment (so their clocks stay close). One video file per camera; the
camera id is parsed from the IP in the filename. Layout for pair `64_65`:

```
c2c_videos/
└── 64_65/
    ├── cam2_192.168.1.64_00000.mkv
    └── cam3_192.168.1.65_00000.mkv
```

**B. Live synchronized snapshots.** If you can reach the cameras live, grab a
frame from both at once and stamp them identically (hold the board still per grab):

```bash
python3 scripts/capture_sync_rtsp.py --out pairs/64_65 --interval 0.5   # or --manual
```

Method B writes the Step-3 folder layout directly — **skip Step 3** and go to Step 4.

---

## 3. Convert the data into calibration format

Kalibr consumes a ROS **bag**; we get there in two sub-steps.

### 3a. Videos → timestamp-named frame pairs

`extract_pairs_from_video.py` turns each pair video into deduped, shared-timestamp
PNGs. It scores every frame (a 48×48 contrast-normalised signature), treats each
3–5 s low-motion run as one **pose**, keeps its **sharpest** frame, and stamps the
two cameras of a pose with an identical synthetic timestamp. It also
cross-correlates the two motion signals to recover any inter-camera offset.

```bash
# tune against the montage first — writes review images + csv, no PNGs:
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --dry-run

# then extract (all pairs found under the root):
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --prune-unpaired
```

Output per pair:
```
pairs/64_65/
├── cam64/<ts_ns>.png     # identical <ts_ns> stems across the two cams
├── cam65/<ts_ns>.png
├── review_montage.jpg    # side-by-side cam64 | cam65 per pose — inspect this
└── index.csv
```

**Cull bad poses:** open `review_montage.jpg`; for any pose that's blurred,
board-not-in-both, or redundant, delete its `<ts>.png` from **both** cam folders
(same stem). Useful knobs: `--min-still` (dwell seconds, default 1.0),
`--dup-thresh` (merge near-identical poses), `--per-pose N` (keep N sharpest),
`--still-thresh` (override the auto motion threshold).

> Only need one pair? `--pair ../c2c_videos/64_65 --out pairs`.

### 3b. Frame pairs → ROS bag (inside Docker)

```bash
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/64_65 --output-bag 64_65.bag'
```

The 19-digit nanosecond filenames become the ROS message timestamps; each
`cam<N>/` folder becomes topic `/cam<N>/image_raw`.

---

## 4. Calibrate

Run the stereo solve. **Topics must be in chain order** (parent first), one
`pinhole-radtan` model per camera. You can fold 3b and this into a single
`docker run`:

```bash
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/64_65 --output-bag 64_65.bag &&
  rosrun kalibr kalibr_calibrate_cameras --bag 64_65.bag \
      --topics /cam64/image_raw /cam65/image_raw \
      --models pinhole-radtan pinhole-radtan \
      --target aprilgrid_6x6.yaml --approx-sync 0.01 --dont-show-report'
```

- `--approx-sync 0.01` is safe: each pose's two frames share an exact timestamp,
  and successive poses are far apart, so views never cross-contaminate.
- Runtime is typically 1–5 min for a pair.

**Outputs** (written next to the bag, in `extrinsics/`):

| File | Contents |
|---|---|
| `64_65-camchain.yaml` | **The result** — per-camera intrinsics + `T_cn_cnm1`. |
| `64_65-results-cam.txt` | Reprojection errors, ±1σ parameter stds, baselines. |
| `64_65-report-cam.pdf` | Diagnostic plots (reprojection scatter, corner coverage). |

---

## 5. Evaluate the result

Never ship a calibration without checking it. Four independent checks, cheapest
first — a result should pass **all** of them.

### 5a. Reprojection error (primary metric)

Open `64_65-results-cam.txt`. Each camera reports:

```
reprojection error: [mean_x, mean_y] +- [std_x, std_y]     # pixels
```

| Metric | Good | Marginal | Reject |
|---|---|---|---|
| per-camera reproj **std** (px) | ≤ 0.3 | 0.3 – 0.5 | > 0.5 |
| reproj **mean** (px) | ~0 (±0.01) | — | large / biased |

For reference, a healthy run on this rig sits at **±0.17–0.28 px**. The mean should
be essentially zero; a non-zero mean means a systematic model mismatch.

### 5b. Parameter uncertainty (±1σ)

The same file prints `+- [...]` after each `projection:` / `distortion:` / baseline
line — the optimizer's 1σ std for each parameter.

- **Focal / principal point** σ should be **sub-pixel** (< ~2 px). Large σ ⇒ that
  parameter is poorly constrained (usually too few / non-diverse poses).
- **Baseline translation** σ should be **≤ a few mm**. If σ on any translation
  component is > ~1 cm, distrust the baseline.

### 5c. Physical sanity of the extrinsics

Convert the transform to human-readable pose and check it against a tape measure:

```bash
python3 scripts/kalibr_to_opencv.py --camchain 64_65-camchain.yaml
```
Prints, per link:
```
cam64 -> cam65: pan(yaw)=-44.93  pitch=-1.03  roll=+0.60 deg | baseline=15.2 cm
```
Sanity checks:
- **Baseline** matches the physical camera spacing (here inner pairs ≈ 15 cm,
  outer ≈ 27 cm) to within ~1 cm.
- **Pan (yaw)** matches the mounting fan-out angle; **pitch & roll** near 0 for a
  level rig (a few degrees is fine, tens of degrees means a wrong pose).
- For a multi-cam chain, `scripts/extrinsics_relative.py --camchain <file> --ref 64`
  tabulates every camera's angle vs a reference — expect a smooth, monotonic fan.

### 5d. Cross-check intrinsics vs the chessboard (optional but recommended)

Kalibr **re-estimates** intrinsics from overlap-only views; compare them to your
independent chessboard intrinsics to catch overfitting:

```bash
python3 scripts/compare_calib.py --camchain 64_65-camchain.yaml
# -> 64_65-comparison-report.md
```
The report tabulates `fx, fy, cx, cy, k1…` for Kalibr vs chessboard with a relative
Δ per parameter.

| Δ (Kalibr vs chessboard) | Verdict |
|---|---|
| `fx/fy` within ~3 %, `cx/cy` within ~2 % | consistent — good |
| larger, or `k1` sign flip / `fx` off by >10 % | one set is overfit — investigate |

**Which set to ship?** Kalibr's intrinsics are self-consistent with the extrinsics
(reproject cleaner together) but come from overlap-only views; the chessboard set
has better full-frame coverage. Default to **Kalibr's** for a stitcher that uses
intrinsics + extrinsics jointly; ship the chessboard set only if you trust its
coverage more and accept minor inconsistency.

### 5e. Visual check (the PDF)

Skim `64_65-report-cam.pdf`:
- **Reprojection scatter** should be a tight, centred cloud (no structure/arcs).
- **Corner-coverage** plots should fill much of each image, not cluster centrally
  — sparse coverage flags where to add poses on a re-capture.

### If it fails

| Symptom | Fix |
|---|---|
| reproj std > 0.5 px | too few / blurred poses → re-capture (Step 2) with more variety. |
| high σ on a parameter | add poses at varied distance & tilt; that parameter is unconstrained. |
| baseline / angle implausible | check topic order (parent first) and that both cams saw the board; re-inspect the montage. |
| Kalibr "0 observations" | target not detected — confirm `--target` matches the physical board; regenerate with `kalibr_create_target_pdf`. |
| intrinsics far from chessboard | overfit from thin coverage — capture full-frame board sweeps or fix intrinsics. |

---

## Command summary (pair `64_65`)

```bash
cd extrinsics
# 3. convert
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --prune-unpaired
# 4. bag + calibrate (Docker)
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/64_65 --output-bag 64_65.bag &&
  rosrun kalibr kalibr_calibrate_cameras --bag 64_65.bag \
    --topics /cam64/image_raw /cam65/image_raw \
    --models pinhole-radtan pinhole-radtan \
    --target aprilgrid_6x6.yaml --approx-sync 0.01 --dont-show-report'
# 5. evaluate
python3 scripts/kalibr_to_opencv.py --camchain 64_65-camchain.yaml
python3 scripts/compare_calib.py   --camchain 64_65-camchain.yaml
```

Recordings, extracted frames, bags, and Kalibr outputs are **gitignored** — they
are per-rig data, regenerated by following this guide. The tracked content is the
method (this guide + `scripts/`).
