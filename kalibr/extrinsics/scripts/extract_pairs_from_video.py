#!/usr/bin/env python3
"""Extract synchronized AprilGrid frame-pairs from recorded RTSP videos for Kalibr.

Context
-------
We have 4 directories under ``c2c_videos/``, one per adjacent camera pair, each
holding two ``.mkv`` recordings (HEVC, 1920x1080, ~25 fps). The operator walked
an AprilGrid through each pair's overlap region, **holding it still 3-5 s at
each pose**. We must turn these videos into the timestamp-named PNG layout that
``kalibr_bagcreater`` consumes, with the two cameras of a pair sharing one
timestamp per pose so Kalibr fuses them into a single synchronized view.

Two problems, two solutions
---------------------------
1. "Reduce frames showing the same AprilGrid position." A 3-5 s dwell at 25 fps
   is 75-125 near-identical frames. We compute a cheap per-frame *signature*
   (a 48x48 contrast-normalized thumbnail) and a *motion signal* = RMS change
   between consecutive signatures. The board moving spikes the signal; a held
   pose is a flat low-motion run (a "dwell"). We keep ONE (the sharpest) frame
   per dwell. See ``--still-thresh`` / ``--min-still`` / ``--dup-thresh``.

2. "Correct relative timestamp" with no hardware sync. The two videos in a pair
   were started together, so their motion signals cross-correlate at lag ~0
   (the script measures and reports it; override with ``--lag``). Because the
   board is *static* during a dwell, exact sync is irrelevant: we pick the
   sharpest frame in each camera within the dwell and stamp BOTH with one
   shared synthetic timestamp -> Kalibr groups them perfectly at
   ``--approx-sync 0.01``.

Manual culling
--------------
This is best-effort: a "dwell" only means the scene was still, not that the
board is in BOTH cameras (with ~30% overlap it sometimes is not). So the script
also writes a side-by-side **review montage** per pair. Eyeball it and delete
any bad pose: the two cameras share the same ``<ts>.png`` filename stem, so
remove that stem from both cam folders (or use ``--prune-unpaired`` to drop any
stem that is not present in both).

Output layout (directly consumable by kalibr_bagcreater)
--------------------------------------------------------
    <out>/<pair>/cam64/<TS_NS>.png
    <out>/<pair>/cam65/<TS_NS>.png
    <out>/<pair>/index.csv
    <out>/<pair>/review_montage.jpg

Usage
-----
    # analyze only, write montage + csv, no PNGs (tune thresholds first):
    python3 extract_pairs_from_video.py --videos-root ../c2c_videos --dry-run

    # extract all four pairs:
    python3 extract_pairs_from_video.py --videos-root ../c2c_videos --out ./pairs

    # one pair, keep 2 sharpest frames per pose:
    python3 extract_pairs_from_video.py --pair ../c2c_videos/64_65 --out ./pairs --per-pose 2

Dependencies: opencv-python, numpy. (No AprilTag detector required.)

!! Detector caveat: generic AprilTag/aruco detectors fail to decode THIS board.
   Before processing all pairs, verify Kalibr's own detector finds the grid --
   run kalibr_calibrate_cameras on one pair and confirm corners are extracted.
"""

import argparse
import csv
import glob
import os
import re
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("opencv (cv2) required: pip install opencv-python")

# Chain order 68 -> 66 -> 64 -> 65 -> 67 (from camera_config.md). Within a pair,
# the camera earlier in this list is written first (parent) for Kalibr topics.
CHAIN = [68, 66, 64, 65, 67]

# Fixed synthetic epoch (2023-11-14) so timestamps are deterministic and have
# the >=10 digits bagcreater needs (secs = ts[:-9], nsecs = ts[-9:]).
BASE_TS_NS = 1_700_000_000 * 10**9


def cam_id_from_path(path):
    """cam64 from '..._192.168.1.64_00000.mkv' (last IP octet)."""
    m = re.search(r"192\.168\.1\.(\d+)", os.path.basename(path))
    if not m:
        m = re.search(r"cam(\d+)", os.path.basename(path))
        if not m:
            raise ValueError("cannot parse camera id from %s" % path)
    return "cam%s" % m.group(1)


def discover_pair(pair_dir):
    """Return [(cam_id, path), (cam_id, path)] ordered by CHAIN position."""
    vids = sorted(glob.glob(os.path.join(pair_dir, "*.mkv")) +
                  glob.glob(os.path.join(pair_dir, "*.mp4")))
    if len(vids) != 2:
        raise ValueError("expected 2 videos in %s, found %d" % (pair_dir, len(vids)))
    items = [(cam_id_from_path(v), v) for v in vids]

    def chain_pos(cam):
        try:
            return CHAIN.index(int(cam[3:]))
        except (ValueError, KeyError):
            return 999
    items.sort(key=lambda it: chain_pos(it[0]))
    return items


def analyze_video(path, step, sig_size):
    """Single decode pass. Returns parallel arrays over sampled frames:
    idx (frame number), sig (sig_size^2 normalized thumb), sharp (Laplacian var).
    """
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        raise IOError("cannot open %s" % path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 25.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    idxs, sigs, sharps = [], [], []
    i = 0
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        if i % step == 0:
            g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
            s = cv2.resize(g, (sig_size, sig_size)).astype(np.float32)
            s = (s - s.mean()) / (s.std() + 1e-6)
            sigs.append(s)
            sharps.append(cv2.Laplacian(g, cv2.CV_64F).var())
            idxs.append(i)
        i += 1
        if total and i % 500 == 0:
            print("    decode %s: %d/%d" % (os.path.basename(path), i, total),
                  end="\r")
    cap.release()
    print(" " * 70, end="\r")
    return {"fps": fps, "idx": np.array(idxs), "sig": np.array(sigs),
            "sharp": np.array(sharps)}


def motion_signal(sigs):
    """RMS change between consecutive signatures (the 'compare' value)."""
    d = np.sqrt(((sigs[1:] - sigs[:-1]) ** 2).mean(axis=(1, 2)))
    return np.concatenate([[0.0], d])


def estimate_lag(ma, mb):
    """Integer sample lag aligning B onto A via motion-signal cross-correlation.
    Returns s such that A[k] corresponds to B[k - s]. (~0 if started together.)
    """
    n = min(len(ma), len(mb))
    a = ma[:n] - ma[:n].mean()
    b = mb[:n] - mb[:n].mean()
    a /= (a.std() + 1e-9)
    b /= (b.std() + 1e-9)
    xc = np.correlate(a, b, mode="full")
    lag = int(xc.argmax() - (n - 1))
    peak = float(xc.max() / n)
    return lag, peak


def auto_still_thresh(motion):
    med = float(np.median(motion))
    spread = float(np.percentile(motion, 90) - med)
    return float(np.clip(med + 0.5 * spread, 0.08, 0.25))


def find_runs(mask, min_len):
    """Contiguous True runs of length >= min_len -> list of (start, end_excl)."""
    runs = []
    start = None
    for i, v in enumerate(mask):
        if v and start is None:
            start = i
        elif not v and start is not None:
            if i - start >= min_len:
                runs.append((start, i))
            start = None
    if start is not None and len(mask) - start >= min_len:
        runs.append((start, len(mask)))
    return runs


def dedup_runs(runs, sigs, dup_thresh):
    """Drop a dwell whose mean signature is within dup_thresh of an earlier kept
    one (operator revisited the same pose)."""
    kept, reps = [], []
    for (k0, k1) in runs:
        rep = sigs[k0:k1].mean(axis=0)
        dup = any(np.sqrt(((rep - r) ** 2).mean()) < dup_thresh for r in reps)
        if not dup:
            kept.append((k0, k1))
            reps.append(rep)
    return kept


def save_gray_frames(path, frame_indices, out_dir, names):
    """Pass 2: walk the video, write requested full-res frames as grayscale PNG.
    frame_indices -> names is a dict {frame_idx: out_filename_stem}."""
    os.makedirs(out_dir, exist_ok=True)
    cap = cv2.VideoCapture(path)
    want = dict(frame_indices)
    i = 0
    saved = 0
    while want:
        ok = cap.grab()
        if not ok:
            break
        if i in want:
            _, fr = cap.retrieve()
            g = cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(os.path.join(out_dir, want[i] + ".png"), g)
            del want[i]
            saved += 1
        i += 1
    cap.release()
    return saved


def thumb_at(path, frame_idx, w=320):
    cap = cv2.VideoCapture(path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, fr = cap.read()
    cap.release()
    if not ok:
        return np.zeros((w * 9 // 16, w, 3), np.uint8)
    h = int(fr.shape[0] * w / fr.shape[1])
    return cv2.resize(fr, (w, h))


def build_montage(pair_dir, picks, names, out_path, cols=4):
    """Side-by-side [camA | camB] thumbnail per pose, labeled. For manual culling."""
    (ida, va), (idb, vb) = pair_dir
    cells = []
    for p in picks:
        ta = thumb_at(va, p["fa"]) ; tb = thumb_at(vb, p["fb"])
        h = max(ta.shape[0], tb.shape[0])
        ta = cv2.copyMakeBorder(ta, 0, h - ta.shape[0], 0, 4, cv2.BORDER_CONSTANT)
        tb = cv2.copyMakeBorder(tb, 0, h - tb.shape[0], 0, 0, cv2.BORDER_CONSTANT)
        cell = np.hstack([ta, tb])
        label = "#%02d %s  shA=%d shB=%d" % (p["pose"], names[p["pose"]],
                                             int(p["sa"]), int(p["sb"]))
        cv2.rectangle(cell, (0, 0), (cell.shape[1], 22), (0, 0, 0), -1)
        cv2.putText(cell, label, (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 0), 1, cv2.LINE_AA)
        cells.append(cell)
    if not cells:
        return
    cw = max(c.shape[1] for c in cells)
    ch = max(c.shape[0] for c in cells)
    cells = [cv2.copyMakeBorder(c, 0, ch - c.shape[0], 0, cw - c.shape[1],
                                cv2.BORDER_CONSTANT) for c in cells]
    rows = []
    for r in range(0, len(cells), cols):
        row = cells[r:r + cols]
        while len(row) < cols:
            row.append(np.zeros_like(cells[0]))
        rows.append(np.hstack(row))
    cv2.imwrite(out_path, np.vstack(rows))


def process_pair(pair_dir_path, out_root, args):
    pair_name = os.path.basename(os.path.normpath(pair_dir_path))
    pair = discover_pair(pair_dir_path)
    (ida, va), (idb, vb) = pair
    print("\n=== pair %s : %s (parent) + %s (child) ===" % (pair_name, ida, idb))

    A = analyze_video(va, args.step, args.sig)
    B = analyze_video(vb, args.step, args.sig)
    ma = motion_signal(A["sig"])
    mb = motion_signal(B["sig"])

    if args.lag is not None:
        lag, peak = args.lag, float("nan")
        print("  using forced lag=%d samples" % lag)
    else:
        lag, peak = estimate_lag(ma, mb)
        print("  cross-corr lag=%d samples (%.2fs)  peak=%.2f"
              % (lag, lag * args.step / A["fps"], peak))

    thr_a = args.still_thresh or auto_still_thresh(ma)
    thr_b = args.still_thresh or auto_still_thresh(mb)
    still_a = ma < thr_a
    still_b = mb < thr_b
    print("  still-thresh A=%.3f B=%.3f" % (thr_a, thr_b))

    # Align: A[k] <-> B[k - lag]. Build still-in-both over valid A indices.
    n = len(still_a)
    both = np.zeros(n, dtype=bool)
    for k in range(n):
        j = k - lag
        if 0 <= j < len(still_b):
            both[k] = still_a[k] and still_b[j]

    min_len = max(2, int(round(args.min_still * A["fps"] / args.step)))
    runs = find_runs(both, min_len)
    runs = dedup_runs(runs, A["sig"], args.dup_thresh)
    print("  dwells (>= %.1fs, deduped): %d" % (args.min_still, len(runs)))

    # Pick the sharpest frame(s) per dwell in each camera independently.
    picks = []
    for pose, (k0, k1) in enumerate(runs):
        ks = np.arange(k0, k1)
        topA = ks[np.argsort(A["sharp"][ks])[::-1][:args.per_pose]]
        for rank, ka in enumerate(topA):
            kb = ka - lag
            if not (0 <= kb < len(B["idx"])):
                continue
            picks.append({
                "pose": pose, "rank": rank,
                "fa": int(A["idx"][ka]), "fb": int(B["idx"][kb]),
                "sa": A["sharp"][ka], "sb": B["sharp"][kb],
            })

    # Shared synthetic timestamps; poses 0.2s apart by default (>> approx-sync).
    names = {}
    for p in picks:
        ts = BASE_TS_NS + (p["pose"] * args.per_pose + p["rank"]) * int(args.interval * 1e9)
        p["ts"] = ts
        names.setdefault(p["pose"], "%d" % ts)
        p["name"] = "%d" % ts

    out_dir = os.path.join(out_root, pair_name)
    os.makedirs(out_dir, exist_ok=True)
    build_montage(pair, picks, {p["pose"]: p["name"] for p in picks},
                  os.path.join(out_dir, "review_montage.jpg"), cols=args.montage_cols)

    with open(os.path.join(out_dir, "index.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pose", "rank", "ts_ns", ida + "_frame", idb + "_frame",
                    "sharp_" + ida, "sharp_" + idb])
        for p in picks:
            w.writerow([p["pose"], p["rank"], p["ts"], p["fa"], p["fb"],
                        "%.1f" % p["sa"], "%.1f" % p["sb"]])

    if args.dry_run:
        print("  [dry-run] %d frame-pairs selected. Montage+csv -> %s"
              % (len(picks), out_dir))
    else:
        a_saved = save_gray_frames(va, {p["fa"]: p["name"] for p in picks},
                                   os.path.join(out_dir, ida), None)
        b_saved = save_gray_frames(vb, {p["fb"]: p["name"] for p in picks},
                                   os.path.join(out_dir, idb), None)
        print("  wrote %d (%s) + %d (%s) PNGs -> %s"
              % (a_saved, ida, b_saved, idb, out_dir))

    # Suggested Kalibr commands (parent topic first = chain order).
    print("  next:")
    print("    kalibr_bagcreater --folder %s --output-bag %s.bag"
          % (out_dir, pair_name))
    print("    kalibr_calibrate_cameras --bag %s.bag \\" % pair_name)
    print("      --topics /%s/image_raw /%s/image_raw \\" % (ida, idb))
    print("      --models pinhole-radtan pinhole-radtan \\")
    print("      --target %s --approx-sync 0.01"
          % os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "aprilgrid_6x6.yaml"))
    return len(picks)


def prune_unpaired(out_root):
    """Drop any timestamp stem not present in BOTH cam folders of a pair."""
    for pair_dir in sorted(glob.glob(os.path.join(out_root, "*"))):
        if not os.path.isdir(pair_dir):
            continue
        cams = sorted(d for d in os.listdir(pair_dir)
                      if d.startswith("cam") and
                      os.path.isdir(os.path.join(pair_dir, d)))
        if len(cams) != 2:
            continue
        stems = []
        for c in cams:
            stems.append({os.path.splitext(f)[0]
                          for f in os.listdir(os.path.join(pair_dir, c))
                          if f.endswith(".png")})
        common = stems[0] & stems[1]
        removed = 0
        for c, st in zip(cams, stems):
            for s in st - common:
                os.remove(os.path.join(pair_dir, c, s + ".png"))
                removed += 1
        if removed:
            print("  pruned %d unpaired PNGs in %s" % (removed, os.path.basename(pair_dir)))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--videos-root", help="dir containing the 4 pair subfolders")
    ap.add_argument("--pair", help="process a single pair dir instead of all")
    ap.add_argument("--out", default="./pairs", help="output root (default ./pairs)")
    ap.add_argument("--step", type=int, default=2,
                    help="analyze every Nth frame (default 2 = ~12.5Hz)")
    ap.add_argument("--sig", type=int, default=48, help="signature thumb size")
    ap.add_argument("--still-thresh", type=float, default=None,
                    help="motion below this = still (default: auto per video)")
    ap.add_argument("--min-still", type=float, default=1.0,
                    help="min dwell duration in seconds (default 1.0)")
    ap.add_argument("--dup-thresh", type=float, default=0.10,
                    help="merge dwells with signature distance below this")
    ap.add_argument("--per-pose", type=int, default=1,
                    help="sharpest frames to keep per dwell (default 1)")
    ap.add_argument("--interval", type=float, default=0.2,
                    help="synthetic seconds between stamped poses (>> approx-sync)")
    ap.add_argument("--lag", type=int, default=None,
                    help="force inter-camera sample lag (skip cross-corr)")
    ap.add_argument("--montage-cols", type=int, default=4)
    ap.add_argument("--dry-run", action="store_true",
                    help="analyze + write montage/csv only, no PNG extraction")
    ap.add_argument("--prune-unpaired", action="store_true",
                    help="after extraction, drop stems not in both cam folders")
    args = ap.parse_args()

    if args.pair:
        pair_dirs = [args.pair]
    elif args.videos_root:
        pair_dirs = sorted(d for d in glob.glob(os.path.join(args.videos_root, "*"))
                           if os.path.isdir(d) and glob.glob(os.path.join(d, "*.mkv")))
    else:
        ap.error("provide --videos-root or --pair")

    total = 0
    for pd in pair_dirs:
        try:
            total += process_pair(pd, args.out, args)
        except (ValueError, IOError) as e:
            print("  SKIP %s: %s" % (pd, e))

    if args.prune_unpaired and not args.dry_run:
        prune_unpaired(args.out)

    print("\n[done] %d frame-pairs across %d pair(s) -> %s"
          % (total, len(pair_dirs), args.out))
    if not args.dry_run:
        print("Review each <pair>/review_montage.jpg; delete bad poses by stem "
              "from BOTH cam folders, then run kalibr_bagcreater.")


if __name__ == "__main__":
    main()
