#!/usr/bin/env python3
"""Create two-video input directories for every adjacent pair and segment."""
import argparse
from pathlib import Path
import re
import sys

PAIRS = (("66", "68"), ("64", "66"), ("64", "65"), ("65", "67"))
SEGMENT_RE = re.compile(r"_(\d+)\.mkv$", re.IGNORECASE)


def list_segments(recording_dir, camera_id):
    camera_dir = recording_dir / ("cam" + camera_id)
    if not camera_dir.is_dir():
        raise ValueError("missing camera directory: {}".format(camera_dir))
    segments = {}
    for path in camera_dir.glob("*.mkv"):
        match = SEGMENT_RE.search(path.name)
        if not match:
            raise ValueError("cannot read segment number from {}".format(path.name))
        number = int(match.group(1))
        if number in segments:
            raise ValueError("duplicate segment {} for cam{}".format(number, camera_id))
        segments[number] = path.resolve()
    if not segments:
        raise ValueError("no .mkv segments found in {}".format(camera_dir))
    return segments


def prepare(recording_dir, out_dir, dry_run=False):
    by_camera = {camera_id: list_segments(recording_dir, camera_id)
                 for camera_id in ("68", "66", "64", "65", "67")}
    segment_sets = {camera_id: set(segments) for camera_id, segments in by_camera.items()}
    expected = set.union(*segment_sets.values())
    for camera_id, numbers in segment_sets.items():
        if numbers != expected:
            raise ValueError("cam{} segments {} do not match expected {}"
                             .format(camera_id, sorted(numbers), sorted(expected)))
    if out_dir.exists() and any(out_dir.iterdir()):
        raise ValueError("output directory is not empty: {}".format(out_dir))
    if not dry_run:
        out_dir.mkdir(parents=True, exist_ok=True)

    for segment in sorted(expected):
        for first, second in PAIRS:
            pair_dir = out_dir / "{}_{}_seg{:03d}".format(first, second, segment)
            if not dry_run:
                pair_dir.mkdir(parents=True, exist_ok=True)
            for camera_id in (first, second):
                source = by_camera[camera_id][segment]
                destination = pair_dir / source.name
                print("{} -> {}".format(source, destination))
                if not dry_run:
                    destination.symlink_to(source)
    return len(expected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording-dir", required=True,
                        help="one run directory produced by record_5cam.sh")
    parser.add_argument("--out", required=True,
                        help="empty directory for pair/segment input folders")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    try:
        count = prepare(Path(args.recording_dir).resolve(),
                        Path(args.out).resolve(), args.dry_run)
    except (OSError, ValueError) as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2
    print("{} segment(s) prepared for {} adjacent camera pairs{}"
          .format(count, len(PAIRS), " (dry run)" if args.dry_run else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
