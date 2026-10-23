# Extrinsic Calibration — 5 RTSP Cameras (Kalibr)

> 中文版本: [`EXTRINSIC_CALIBRATION_GUIDE_zh.md`](EXTRINSIC_CALIBRATION_GUIDE_zh.md)

Calibrate the extrinsics of the 5-camera surround rig
(chain `68 → 66 → 64 → 65 → 67`, each camera sharing ~30 % FOV with its
neighbour) with Kalibr — **run entirely on your local machine** — in four steps:

```
 1. record 5 ──▶ 2. build frame set ──▶ 3. calibrate (local Docker) ──▶ 4. export YAML
   cameras      prepare pairs + extract    bagcreater + calibrate         opencv / apollo
                → one 5-cam folder         → *-camchain.yaml
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
- On the recording host: **PyGObject** and GStreamer RTSP, H.264/H.265, and Matroska plugins.

---

## Step 1 — Record all five cameras

This repo includes a standalone recorder; it does not depend on `video-stitch`.
Run the recorder and its per-camera GStreamer pipelines on a host that can reach
the cameras. Kalibr remains in the local Docker image (Step 3). Copy the example
config and adjust the camera endpoints, RTSP paths, and codecs:

```bash
cp cameras.example.json cameras.local.json
```

Do not put RTSP credentials in the config or commit them. Supply them through
environment variables:

```bash
export RTSP_USER='your-camera-user'
export RTSP_PASSWORD='your-camera-password'
./scripts/record_5cam.sh --duration 180 --length 3600
```

Recordings are written to `recordings/<run_id>/cam<id>/`; each filename includes
the camera ID, IP, and segment number. Per-frame clock values are kept in a
`.frames.csv` sidecar for reference only; the current Kalibr workflow does not
use those timestamps for observation pairing. The default segment length is one
hour, so a shorter run normally produces one video per camera. Ctrl-C and
`--duration` both stop via EOS so Matroska segments can finalize.

If the run has multiple segments, all five cameras must have matching segment
numbers. Prepare the four neighboring-pair inputs (symlinks to the recordings):

```bash
python3 scripts/prepare_video_pairs.py \
  --recording-dir recordings/<run_id> --out pairs/video_inputs
```

## Step 2 — Build the frame set

The five simultaneous recordings provide the four adjacent camera pairs. Operator protocol:

- Keep the AprilGrid in the pair's **overlap region**, visible to **both** cameras.
- **Stop-and-go:** move, **hold still 3–5 s**, move. Stillness is what removes the
  need for hardware sync.
- ~**20–30 distinct poses** per pair across varied **distance, tilt, position**.
- Avoid motion blur; keep the board reasonably large.

Prepared pair layout (camera id parsed from the IP in each filename):

```
pairs/video_inputs/
├── 66_68_seg000/  *_192.168.1.68_*.mkv  *_192.168.1.66_*.mkv
├── 64_66_seg000/  *_192.168.1.66_*.mkv  *_192.168.1.64_*.mkv
├── 64_65_seg000/  *_192.168.1.64_*.mkv  *_192.168.1.65_*.mkv
└── 65_67_seg000/  *_192.168.1.65_*.mkv  *_192.168.1.67_*.mkv
```

You may also record each adjacent pair separately and feed its two videos
directly into the extraction step below. For live synchronized snapshots,
`scripts/capture_sync_rtsp.py` remains available (see
[Appendix](#appendix--live-snapshot-capture)).

**Extract** deduped, shared-timestamp frame-pairs from every pair video. The tool
detects each 3–5 s dwell (a low-motion run of a per-frame signature) and keeps the
**sharpest** frame, stamping both cameras of a pose identically.

```bash
for pair in pairs/video_inputs/*; do
  python3 scripts/extract_pairs_from_video.py --pair "$pair" \
    --out pairs/extracted_parts --prune-unpaired
done
```

Open each `pairs/extracted_parts/<pair>_segNNN/review_montage.jpg` (side-by-side cam A | cam B per pose)
and delete any bad pose's `<ts>.png` from **both** cam folders. Knobs: `--min-still`,
`--dup-thresh`, `--per-pose`, `--still-thresh` (or `--dry-run` to tune first).

Merge the extracted segment folders belonging to each camera pair. For a
single-segment recording:

```bash
python3 scripts/merge_pairs.py --out pairs/66_68 --source pairs/extracted_parts/66_68_seg000 0
python3 scripts/merge_pairs.py --out pairs/64_66 --source pairs/extracted_parts/64_66_seg000 0
python3 scripts/merge_pairs.py --out pairs/64_65 --source pairs/extracted_parts/64_65_seg000 0
python3 scripts/merge_pairs.py --out pairs/65_67 --source pairs/extracted_parts/65_67_seg000 0
```

For multiple segments, add one `--source <pair>_segNNN <offset>` per segment;
choose offsets so their synthetic timestamp ranges do not overlap.

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
- `scripts/record_5cam.sh` and `scripts/record_cam_synced.py` — five-camera segmented recording with timestamp sidecars.
- `scripts/prepare_video_pairs.py` — organize five-camera recordings into pair inputs for the existing extractor.
- `scripts/kalibr_to_opencv.py` — camchain → OpenCV intrinsics + extrinsics YAML.
- `scripts/extrinsics_relative.py` — per-camera yaw/roll/pitch vs a reference.
- `scripts/compare_calib.py` — Kalibr vs chessboard intrinsics comparison.
- `../intrinsics/intrinsics_to_camchain.py` — intrinsics → apollo `camchain_<id>.yaml`.
- `aprilgrid_6x6.yaml` — Kalibr target definition.

Recordings, sidecars, extracted frames, bags, and Kalibr outputs are **gitignored**
(per-rig data, regenerated by following this guide). The host recorder requires
PyGObject and GStreamer RTSP, H.264/H.265, and Matroska plugins; see
`cameras.example.json` and `scripts/record_5cam.sh --help`.
