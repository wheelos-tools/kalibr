#!/usr/bin/env python3
"""
Merge several image-pair folders into one multi-camera folder for a JOINT Kalibr
calibration (e.g. build a 3-camera group from two pairwise captures that share a
common "bridge" camera).

Each source pair looks like:  <pair>/cam<ID>/<timestamp_ns>.png
Pairs captured in separate sessions reuse the same timestamps, so frames of the
shared camera collide. We add a per-source time OFFSET (seconds) to every file in
that source, placing each session in a disjoint time window. The bridge camera
(present in >1 source) then accumulates frames from every window without collision,
which is exactly what links the cameras in Kalibr's extrinsic chain.

Usage:
  python3 merge_pairs.py --out pairs/66_64_65 \
      --source pairs/64_65 0 \
      --source pairs/64_66 10
"""
import argparse
import os
import shutil

IMG_EXT = (".png", ".jpg", ".jpeg", ".bmp")
NS_PER_S = 1_000_000_000


def cam_dirs(pair_dir):
    return sorted(d for d in os.listdir(pair_dir)
                  if d.startswith("cam") and os.path.isdir(os.path.join(pair_dir, d)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output multi-camera folder")
    ap.add_argument("--source", required=True, nargs=2, action="append",
                    metavar=("DIR", "OFFSET_S"),
                    help="a source pair dir and its time offset in seconds (repeatable)")
    ap.add_argument("--dry-run", action="store_true", help="report only, copy nothing")
    args = ap.parse_args()

    sources = [(d, int(round(float(s) * NS_PER_S))) for d, s in args.source]

    if not args.dry_run:
        os.makedirs(args.out, exist_ok=True)

    # cam_id -> list of (new_ts_ns, src_path)
    plan = {}
    appears_in = {}  # cam_id -> set of source dirs (to find the bridge)
    for pair_dir, off in sources:
        for cam in cam_dirs(pair_dir):
            appears_in.setdefault(cam, set()).add(pair_dir)
            csrc = os.path.join(pair_dir, cam)
            for f in sorted(os.listdir(csrc)):
                stem, ext = os.path.splitext(f)
                if ext.lower() not in IMG_EXT:
                    continue
                try:
                    ts = int(stem)
                except ValueError:
                    print(f"  skip non-timestamp file: {os.path.join(csrc, f)}")
                    continue
                plan.setdefault(cam, []).append((ts + off, os.path.join(csrc, f), ext))

    # detect collisions within each output cam folder
    for cam, items in plan.items():
        seen = {}
        for new_ts, src, _ in items:
            if new_ts in seen:
                raise SystemExit(
                    f"COLLISION in {cam}: {src} and {seen[new_ts]} both map to "
                    f"{new_ts} ns — increase an offset.")
            seen[new_ts] = src

    # execute + summarize
    print(f"output: {args.out}")
    for cam in sorted(plan):
        items = sorted(plan[cam])
        tmin, tmax = items[0][0], items[-1][0]
        bridge = " (BRIDGE)" if len(appears_in[cam]) > 1 else ""
        print(f"  {cam}{bridge}: {len(items):3d} frames | "
              f"window {tmin/NS_PER_S:.3f}..{tmax/NS_PER_S:.3f} s")
        if args.dry_run:
            continue
        cdst = os.path.join(args.out, cam)
        os.makedirs(cdst, exist_ok=True)
        for new_ts, src, ext in items:
            shutil.copy2(src, os.path.join(cdst, f"{new_ts}{ext}"))

    # suggest the Kalibr topic order: leaf, bridge, leaf  (bridge in the middle)
    bridges = [c for c in plan if len(appears_in[c]) > 1]
    leaves = [c for c in plan if len(appears_in[c]) == 1]
    if len(bridges) == 1 and len(leaves) == 2:
        order = [leaves[0], bridges[0], leaves[1]]
        topics = " ".join(f"/{c}/image_raw" for c in order)
        print(f"\nsuggested --topics order (bridge in the middle): {topics}")
    print("\ndone." if not args.dry_run else "\n(dry run — nothing copied)")


if __name__ == "__main__":
    main()
