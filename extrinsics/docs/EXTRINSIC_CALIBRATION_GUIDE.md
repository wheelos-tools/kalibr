# Extrinsic Calibration — 5 RTSP Cameras (Kalibr)

Calibrate the extrinsics of the 5-camera surround rig
(chain `68 → 66 → 64 → 65 → 67`, each camera sharing ~30 % FOV with its
neighbour) with Kalibr — **run entirely on your local machine** — in four steps:

```
 1. record ──▶ 2. build frame set ──▶ 3. calibrate (local Docker) ──▶ 4. export YAML
   pair         extract + merge          bagcreater + calibrate         opencv / apollo
   videos       → one 5-cam folder       → *-camchain.yaml
```

All commands below are run **from the `extrinsics/` directory**; scripts live in
`scripts/`, the Kalibr target is `aprilgrid_6x6.yaml`.

### Why this works without hardware sync
The cameras are independent RTSP streams with no trigger (50–150 ms jitter).
Extrinsics are a **static** transform, so we hold the board still per pose and
give a pose's two camera frames **one identical synthetic timestamp** — Kalibr
fuses them into one view at `--approx-sync 0.01`. The AprilGrid's unique tag IDs
make the 30 % overlap workable (partial views still count).

> ⚠️ Generic Python AprilTag detectors (`pupil-apriltags`, OpenCV `aruco`) do
> **not** decode this board, but **Kalibr's own detector does** (results are
> sub-pixel). Verify detection via a Kalibr run, not a Python check.

### Prerequisites
- Per-camera **intrinsics** already calibrated → `../intrinsics/intrinsics_cam*.yaml`
  (Kalibr re-estimates them anyway; see [Step 4](#step-4--export-to-yaml)).
- **Docker**, and the Kalibr image built locally (Step 3).
- **Python 3** with `opencv-python`, `numpy`, `pyyaml`.

---

## Step 1 — Record one video per adjacent pair

Four pairs for a 5-camera chain. Run both cameras of a pair at once (start them
together so their clocks stay close). Operator protocol:

- Keep the AprilGrid in the pair's **overlap region**, visible to **both** cameras.
- **Stop-and-go:** move, **hold still 3–5 s**, move. Stillness is what removes the
  need for hardware sync.
- ~**20–30 distinct poses** per pair across varied **distance, tilt, position**.
- Avoid motion blur; keep the board reasonably large.

Layout (camera id parsed from the IP in each filename):

```
c2c_videos/
├── 66_68/  *_192.168.1.68_*.mkv  *_192.168.1.66_*.mkv
├── 64_66/  *_192.168.1.66_*.mkv  *_192.168.1.64_*.mkv
├── 64_65/  *_192.168.1.64_*.mkv  *_192.168.1.65_*.mkv
└── 65_67/  *_192.168.1.65_*.mkv  *_192.168.1.67_*.mkv
```

*(No recorder? `scripts/capture_sync_rtsp.py` grabs synced live snapshots instead —
see [Appendix](#appendix--live-snapshot-capture). The rest of the flow is identical.)*

---

## Step 2 — Build the frame set

**Extract** deduped, shared-timestamp frame-pairs from every pair video. The tool
detects each 3–5 s dwell (a low-motion run of a per-frame signature) and keeps the
**sharpest** frame, stamping both cameras of a pose identically.

```bash
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --prune-unpaired
```

Open each `pairs/<pair>/review_montage.jpg` (side-by-side cam A | cam B per pose)
and delete any bad pose's `<ts>.png` from **both** cam folders. Knobs: `--min-still`,
`--dup-thresh`, `--per-pose`, `--still-thresh` (or `--dry-run` to tune first).

**Merge** all four pairs into one multi-camera folder for a single joint solve.
Because the pairs were recorded separately, the shared "bridge" cameras (66, 64,
65) reuse timestamps; a per-source time offset places each session in its own
window so the bridges link the chain:

```bash
python3 scripts/merge_pairs.py --out pairs/68_66_64_65_67 \
  --source pairs/66_68 0 --source pairs/64_66 10 \
  --source pairs/64_65 20 --source pairs/65_67 30
```

---

## Step 3 — Calibrate with Kalibr (local Docker)

Build the Kalibr image once (from the repo root — `Dockerfile_ros1_20_04` ships
with this repo):

```bash
( cd .. && docker build -t kalibr -f Dockerfile_ros1_20_04 . )
```

Then build the bag and calibrate in **one local container run**, mounting
`extrinsics/` as `/data` (topics in chain order, one `pinhole-radtan` per camera):

```bash
docker run --rm --entrypoint bash -v "$PWD":/data kalibr -c '
  source /catkin_ws/devel/setup.bash && cd /data &&
  rosrun kalibr kalibr_bagcreater --folder pairs/68_66_64_65_67 \
      --output-bag 68_66_64_65_67.bag &&
  rosrun kalibr kalibr_calibrate_cameras --bag 68_66_64_65_67.bag \
      --topics /cam68/image_raw /cam66/image_raw /cam64/image_raw /cam65/image_raw /cam67/image_raw \
      --models pinhole-radtan pinhole-radtan pinhole-radtan pinhole-radtan pinhole-radtan \
      --target aprilgrid_6x6.yaml --approx-sync 0.01 --dont-show-report'
```

`--approx-sync 0.01` is safe (each pose's cameras share an exact stamp; poses are
far apart). Drop `--dont-show-report` to see the PDF GUI if you have X11.

**Outputs** (appear in `extrinsics/`):
- `68_66_64_65_67-camchain.yaml` — **the result**: per-camera intrinsics + `T_cn_cnm1`
  (4×4 mapping a point from cam *n−1* into cam *n*).
- `68_66_64_65_67-results-cam.txt` — reprojection errors + baselines.
- `68_66_64_65_67-report-cam.pdf` — diagnostic plots.

**What good looks like** (this rig): reprojection **±0.19–0.28 px** on every camera;
inner baselines (66↔64, 64↔65) ≈ **15 cm**, outer (68↔66, 65↔67) ≈ **27 cm**;
cameras fan **±90°** around cam64. A bad camera → re-check its poses in the Step 2
montage and recapture that pair.

> On a Linux host the files are written `root`-owned; add `--user $(id -u):$(id -g)`
> to the `docker run`, or `chown` them afterwards. (Not an issue on Docker Desktop.)

---

## Step 4 — Export to YAML

```bash
# (a) camchain -> per-camera OpenCV intrinsics + readable extrinsics (4x4 + quat + euler + baseline)
python3 scripts/kalibr_to_opencv.py --camchain 68_66_64_65_67-camchain.yaml

# (b) per-camera angles relative to a reference camera
python3 scripts/extrinsics_relative.py --camchain 68_66_64_65_67-camchain.yaml --ref 64

# (c) intrinsics -> apollo-lite video-stitch camchain_<id>.yaml (OpenCV FileStorage)
python3 ../intrinsics/intrinsics_to_camchain.py
```

For the apollo-lite stitcher: the per-camera transforms go into
`params_hk/extrinsics_override.yaml`; the `camchain_<id>.yaml` files carry
intrinsics with an identity/blank rotation.

**Which intrinsics to ship?** Kalibr's (from the camchain) are self-consistent with
the extrinsics but estimated from overlap-only views; the chessboard set
(`../intrinsics/`) has better full-frame coverage. They differ ~3 % — run
`scripts/compare_calib.py --camchain <file>` to quantify before deciding.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Python AprilTag detector finds 0 tags | Expected — only Kalibr's detector works on this board. |
| Kalibr: "0 observations" | Target genuinely undetected — check `--target` matches the board; regenerate with `kalibr_create_target_pdf`. |
| One camera has high reprojection error | Too few / non-diverse poses → recapture that pair (vary distance & tilt). |
| `merge_pairs.py` timestamp collision | Increase a `--source` offset above one session's duration. |
| Few poses extracted | Lower `--min-still` (e.g. 0.6) or raise `--dup-thresh`; record longer. |

## Appendix — live-snapshot capture

Instead of recording videos, `scripts/capture_sync_rtsp.py` grabs one frame from
every camera at once and stamps them identically (hold the board still per grab):

```bash
python3 scripts/capture_sync_rtsp.py --out pairs/run01 --interval 0.5   # or --manual
```
Output is already in the Step 3 folder layout; skip Step 2 and go to Step 3.

## File index

- `scripts/extract_pairs_from_video.py` — videos → deduped, shared-timestamp frame-pairs.
- `scripts/merge_pairs.py` — combine pair folders into one multi-cam folder (time-offset bridge).
- `scripts/capture_sync_rtsp.py` — live-snapshot capture alternative.
- `scripts/kalibr_to_opencv.py` — camchain → OpenCV intrinsics + extrinsics YAML.
- `scripts/extrinsics_relative.py` — per-camera yaw/roll/pitch vs a reference.
- `scripts/compare_calib.py` — Kalibr vs chessboard intrinsics comparison.
- `../intrinsics/intrinsics_to_camchain.py` — intrinsics → apollo `camchain_<id>.yaml`.
- `aprilgrid_6x6.yaml` — Kalibr target definition.

Recordings, extracted frames, bags, and Kalibr outputs are **gitignored** (per-rig
data, regenerated by following this guide).
