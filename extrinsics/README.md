# Extrinsic Calibration — 5 RTSP Cameras (Kalibr)

Reusable method to calibrate the extrinsics of a 5-camera surround rig
(chain `68 → 66 → 64 → 65 → 67`, ~30 % adjacent overlap) with Kalibr, run
locally — from AprilGrid pair videos to exported calibration YAML.

➡️ **Full procedure: [`docs/EXTRINSIC_CALIBRATION_GUIDE.md`](docs/EXTRINSIC_CALIBRATION_GUIDE.md)**

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
# 1. extract synced frame-pairs from the pair videos, then merge into one 5-cam folder
python3 scripts/extract_pairs_from_video.py --videos-root ../c2c_videos --out pairs --prune-unpaired
python3 scripts/merge_pairs.py --out pairs/68_66_64_65_67 \
  --source pairs/66_68 0 --source pairs/64_66 10 --source pairs/64_65 20 --source pairs/65_67 30

# 2. calibrate in a local Kalibr container (see the guide for the docker build/run)

# 3. export
python3 scripts/kalibr_to_opencv.py     --camchain 68_66_64_65_67-camchain.yaml
python3 scripts/extrinsics_relative.py  --camchain 68_66_64_65_67-camchain.yaml --ref 64
```

Recordings, extracted frames, bags, and Kalibr outputs are **gitignored** — they
are produced per-rig, not shipped. The tracked content is the method (guide + scripts).
