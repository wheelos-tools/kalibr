# Extrinsic Calibration — 5 RTSP Cameras (Kalibr)

Reusable method to calibrate the extrinsics of a 5-camera surround rig
(chain `68 → 66 → 64 → 65 → 67`, ~30 % adjacent overlap) with Kalibr, run
locally — from AprilGrid pair videos to exported calibration YAML.

➡️ **Multi-camera procedure: [`docs/EXTRINSIC_CALIBRATION_GUIDE.md`](docs/EXTRINSIC_CALIBRATION_GUIDE.md)** · 中文：[`docs/EXTRINSIC_CALIBRATION_GUIDE_zh.md`](docs/EXTRINSIC_CALIBRATION_GUIDE_zh.md)

➡️ **Pairwise camera-to-camera (install → capture → convert → calibrate → evaluate): [`docs/CAMERA2CAMERA_CALIBRATION_GUIDE.md`](docs/CAMERA2CAMERA_CALIBRATION_GUIDE.md)** · 中文：[`docs/CAMERA2CAMERA_CALIBRATION_GUIDE_zh.md`](docs/CAMERA2CAMERA_CALIBRATION_GUIDE_zh.md)

## Layout

```
extrinsics/
├── docs/                 # the guide
├── scripts/              # extraction / merge / export tools (run from extrinsics/)
├── aprilgrid_6x6.yaml    # Kalibr target definition
└── (pairs/, *.bag, *-camchain.yaml, … — per-rig data/results, gitignored)
```

## Quick path (run from `extrinsics/`)

```bash
# 1. configure credentials in the environment, then record all five cameras
export RTSP_USER='your-camera-user'
export RTSP_PASSWORD='your-camera-password'
./scripts/record_5cam.sh --duration 180 --length 3600

# 2. prepare four adjacent camera-pair inputs from the completed recording
python3 scripts/prepare_video_pairs.py \
  --recording-dir recordings/<run_id> --out pairs/video_inputs

# 3. extract synchronized still-pose pairs and inspect the review montages
for pair in pairs/video_inputs/*; do
  python3 scripts/extract_pairs_from_video.py --pair "$pair" \
    --out pairs/extracted_parts --prune-unpaired
done

# 4. merge extracted sessions for each pair, then the four pairs into one rig
python3 scripts/merge_pairs.py --out pairs/66_68 \
  --source pairs/extracted_parts/66_68_seg000 0
python3 scripts/merge_pairs.py --out pairs/64_66 \
  --source pairs/extracted_parts/64_66_seg000 0
python3 scripts/merge_pairs.py --out pairs/64_65 \
  --source pairs/extracted_parts/64_65_seg000 0
python3 scripts/merge_pairs.py --out pairs/65_67 \
  --source pairs/extracted_parts/65_67_seg000 0
python3 scripts/merge_pairs.py --out pairs/68_66_64_65_67 \
  --source pairs/66_68 0 --source pairs/64_66 10 --source pairs/64_65 20 --source pairs/65_67 30

# 5. calibrate in a local Kalibr container (see the guide for the docker build/run)

# 6. export
python3 scripts/kalibr_to_opencv.py     --camchain 68_66_64_65_67-camchain.yaml
python3 scripts/extrinsics_relative.py  --camchain 68_66_64_65_67-camchain.yaml --ref 64
```

Recorder configuration is in `cameras.example.json`; copy it to the ignored
`cameras.local.json` before editing camera endpoints; the recorder uses that
local file automatically when present. Credentials are supplied
through `RTSP_USER` and `RTSP_PASSWORD` and are not stored in the repository.
The recorder and its GStreamer/PyGObject dependencies run on the host that can
reach the cameras; Kalibr still runs in the local Docker image.

The default one-hour segment length avoids splitting a typical calibration run.
If a run produces multiple segments, `prepare_video_pairs.py` creates one
two-video input directory per adjacent pair and segment. Extract each directory
as above, then pass all corresponding `*_segNNN` extracted directories to one
`merge_pairs.py --source` invocation with non-overlapping time offsets before
running the four-pair rig merge.

Recordings, extracted frames, bags, and Kalibr outputs are **gitignored** — they
are produced per-rig, not shipped. The tracked content is the method (guide + scripts).
