#!/usr/bin/env python3
"""Synchronized snapshot capture from several RTSP cameras using GStreamer.

Why this exists
---------------
Kalibr has no live-capture tool: it only consumes a ROS bag (or a folder of
timestamp-named PNGs fed through ``kalibr_bagcreater``). It groups multi-camera
observations into one "view" by header timestamp, within ``--approx-sync``
(default 20 ms). Independent Hikvision RTSP cameras are NOT hardware-synced, so
we cannot land frames within 20 ms of each other in real time.

We sidestep that: extrinsics are a *static* transform, so we hold the board
still, grab the latest frame from every camera at once, and stamp ALL of them
with a SINGLE shared timestamp. The scene is static during the grab, so the
cameras' capture-time skew does not matter, and the identical timestamp makes
Kalibr fuse them into one perfectly synchronized view.

Output layout (directly consumable by kalibr_bagcreater)
--------------------------------------------------------
    <out>/cam68/<TS_NS>.png
    <out>/cam66/<TS_NS>.png
    ...
    <out>/capture_index.csv      # snapshot_idx, ts_ns, per-cam ok/miss

The same <TS_NS> stem is reused across all cameras in one snapshot, so the
files also satisfy tools that pair by "matching filename stems"
(e.g. whl-cal/camera2camera's camera2camera-calibrate).

Run ON THE ORIN (it can reach 192.168.1.x and has HW H.265 decode):
    python3 capture_sync_rtsp.py --out ./run01 --interval 0.5
    python3 capture_sync_rtsp.py --out ./run01 --manual        # press Enter per pose

Dependencies: python3-gi (PyGObject), gstreamer1.0 plugins, numpy, opencv.
"""

import argparse
import csv
import os
import sys
import time

import numpy as np

import gi
gi.require_version("Gst", "1.0")
from gi.repository import Gst  # noqa: E402

# ---- Camera list (from camera_config.md; chain order 68->66->64->65->67) ----
DEFAULT_CAMERAS = [
    {"id": "cam68", "uri": "rtsp://admin:Nvidia135@192.168.1.68:554/live"},
    {"id": "cam66", "uri": "rtsp://admin:Nvidia135@192.168.1.66:554/live"},
    {"id": "cam64", "uri": "rtsp://admin:Nvidia135@192.168.1.64:554/live"},
    {"id": "cam65", "uri": "rtsp://admin:Nvidia135@192.168.1.65:554/live"},
    {"id": "cam67", "uri": "rtsp://admin:Nvidia135@192.168.1.67:554/live"},
]


def have_element(name):
    return Gst.ElementFactory.find(name) is not None


def pick_decoder(codec, mode):
    """Return (depay, parse, decode, postconv_prefix) element strings.

    postconv_prefix lands the buffer in system memory ready for a final
    videoconvert to GRAY8/BGR.
    """
    depay = "rtph265depay" if codec == "h265" else "rtph264depay"
    parse = "h265parse" if codec == "h265" else "h264parse"

    use_jetson = mode == "jetson" or (mode == "auto" and have_element("nvv4l2decoder"))
    if use_jetson:
        # HW decode stays in NVMM; nvvidconv copies it to system memory.
        decode = "nvv4l2decoder ! nvvidconv"
        post = "video/x-raw, format=BGRx ! videoconvert"
    else:
        sw = "avdec_h265" if codec == "h265" else "avdec_h264"
        decode = "{} ! videoconvert".format(sw)
        post = "videoconvert"
    return depay, parse, decode, post


def build_pipeline(cam, codec, decmode, gray, latency):
    depay, parse, decode, post = pick_decoder(codec, decmode)
    out_fmt = "GRAY8" if gray else "BGR"
    # protocols=tcp avoids UDP packet loss artifacts on busy networks.
    desc = (
        "rtspsrc location={uri} protocols=tcp latency={lat} ! "
        "{depay} ! {parse} ! {decode} ! {post} ! "
        "video/x-raw, format={fmt} ! "
        "appsink name=sink emit-signals=false sync=false max-buffers=1 drop=true"
    ).format(uri=cam["uri"], lat=latency, depay=depay, parse=parse,
             decode=decode, post=post, fmt=out_fmt)
    pipe = Gst.parse_launch(desc)
    sink = pipe.get_by_name("sink")
    return pipe, sink


def sample_to_ndarray(sample, gray):
    """Stride-safe conversion of a GstSample to a numpy image."""
    buf = sample.get_buffer()
    s = sample.get_caps().get_structure(0)
    w = s.get_value("width")
    h = s.get_value("height")
    ch = 1 if gray else 3
    ok, info = buf.map(Gst.MapFlags.READ)
    if not ok:
        return None
    try:
        data = np.frombuffer(info.data, dtype=np.uint8)
    finally:
        buf.unmap(info)
    expected = w * h * ch
    if data.size == expected:
        return data.reshape(h, w) if gray else data.reshape(h, w, ch)
    # Handle row padding (stride > w*ch).
    stride = data.size // h
    rows = data[: stride * h].reshape(h, stride)
    if gray:
        return np.ascontiguousarray(rows[:, :w])
    return np.ascontiguousarray(rows[:, : w * ch].reshape(h, w, ch))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="output root dir")
    ap.add_argument("--codec", default="h265", choices=["h265", "h264"])
    ap.add_argument("--decoder", default="auto", choices=["auto", "jetson", "sw"],
                    help="auto picks nvv4l2decoder if present, else software")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="seconds between snapshots (timed mode)")
    ap.add_argument("--manual", action="store_true",
                    help="press Enter to take each snapshot instead of timed")
    ap.add_argument("--max", type=int, default=0,
                    help="stop after N snapshots (0 = until Ctrl+C)")
    ap.add_argument("--color", action="store_true",
                    help="save BGR PNGs (default: GRAY8, matches Kalibr mono8)")
    ap.add_argument("--latency", type=int, default=200,
                    help="rtspsrc jitter buffer ms")
    ap.add_argument("--warmup", type=float, default=3.0,
                    help="seconds to let streams come up before capturing")
    args = ap.parse_args()

    try:
        import cv2
    except ImportError:
        sys.exit("opencv (cv2) is required to write PNGs: pip3 install opencv-python")

    gray = not args.color
    Gst.init(None)

    cams = DEFAULT_CAMERAS
    pipes = []
    for cam in cams:
        os.makedirs(os.path.join(args.out, cam["id"]), exist_ok=True)
        pipe, sink = build_pipeline(cam, args.codec, args.decoder, gray, args.latency)
        pipe.set_state(Gst.State.PLAYING)
        pipes.append((cam, pipe, sink))
        print("[start] {}  {}".format(cam["id"], cam["uri"]))

    print("[warmup] waiting {:.1f}s for streams...".format(args.warmup))
    time.sleep(args.warmup)

    index_path = os.path.join(args.out, "capture_index.csv")
    idx_file = open(index_path, "w", newline="")
    writer = csv.writer(idx_file)
    writer.writerow(["snapshot_idx", "ts_ns"] + [c["id"] for c in cams])

    n = 0
    print("[ready] {} mode. Hold the board STILL at each pose. Ctrl+C to stop."
          .format("MANUAL (Enter)" if args.manual else
                  "TIMED every {:.2f}s".format(args.interval)))
    try:
        while True:
            if args.manual:
                try:
                    input("  press Enter for snapshot {} (Ctrl+C to stop)... "
                          .format(n))
                except EOFError:
                    break
            else:
                time.sleep(args.interval)

            # ONE shared timestamp for the whole snapshot -> Kalibr fuses them.
            ts_ns = int(time.time() * 1e9)
            stem = "{:d}".format(ts_ns)
            row = [n, ts_ns]
            got = 0
            for cam, _pipe, sink in pipes:
                sample = sink.try_pull_sample(Gst.SECOND // 5)  # 200 ms
                if sample is None:
                    row.append("miss")
                    continue
                img = sample_to_ndarray(sample, gray)
                if img is None:
                    row.append("miss")
                    continue
                path = os.path.join(args.out, cam["id"], stem + ".png")
                cv2.imwrite(path, img)
                row.append("ok")
                got += 1
            writer.writerow(row)
            idx_file.flush()
            print("  snapshot {:04d}  ts={}  {}/{} cams"
                  .format(n, stem, got, len(cams)))
            n += 1
            if args.max and n >= args.max:
                break
    except KeyboardInterrupt:
        print("\n[stop] interrupted")
    finally:
        idx_file.close()
        for _cam, pipe, _sink in pipes:
            pipe.set_state(Gst.State.NULL)
        print("[done] {} snapshots -> {}".format(n, args.out))
        print("       index: {}".format(index_path))


if __name__ == "__main__":
    main()
