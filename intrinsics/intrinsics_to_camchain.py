#!/usr/bin/env python3
"""Convert kalibr/whl-cal ``intrinsics_cam*.yaml`` into the apollo-lite
video-stitch ``camchain_*.yaml`` (OpenCV FileStorage) format, one file per
camera, named by physical camera id (camchain_64.yaml ...).

Mapping
-------
  focal            <- fx
  D  [k1,k2,p1,p2] <- distortion_coefficients[0:4]   (k3 dropped: target is 4-param radtan)
  KMat / K         <- camera_matrix  [fx 0 cx; 0 fy cy; 0 0 1]
  RMat / R / EYEMat = identity   (extrinsic left BLANK -- the camera-to-camera
                                  transforms belong in extrinsics_override.yaml
                                  once the kalibr extrinsic calibration is done)
  resolution       <- [image_width, image_height]
  rostopic         = "/cam<id>/image_raw"

Usage
-----
  python3 intrinsics_to_camchain.py            # uses the default paths below
  python3 intrinsics_to_camchain.py --src DIR --out DIR
"""

import argparse
import glob
import os
import re

import yaml

DEFAULT_SRC = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = ("/Users/jie/Work/01code/apollo-lite/modules/"
               "video-stitch/stitching/params_hk")

# Mirrors params_hk/camchain_0.yaml byte-structure; only values are substituted.
TEMPLATE = """%YAML:1.0
---
cam_overlaps: []
camera_model: "pinhole"
# 焦距通常取 fx 和 fy 的均值，或直接取 fx
focal: {focal}
# 畸变参数直接映射：k1, k2, p1, p2
D: [{k1}, {k2}, {p1}, {p2}]
KMat: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ {fx}, 0., {cx},
           0., {fy}, {cy},
           0., 0., 1. ]
distortion_model: radtan
RMat: !!opencv-matrix
   rows: 3
   cols: 3
   dt: f
   data: [ 1, 0, 0, 0, 1, 0, 0, 0, 1 ]
EYEMat: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 1, 0, 0, 0, 1, 0, 0, 0, 1 ]
K: [ {fx}, 0., {cx}, 0., {fy}, {cy}, 0., 0., 1. ]
R: [ 1, 0, 0, 0, 1, 0, 0, 0, 1 ]
resolution: [ {w}, {h} ]
rostopic: "/cam{nn}/image_raw"
"""


def num(v):
    """Round-trip float repr (full precision; no needless trailing zeros)."""
    return repr(float(v))


def convert(src_path, out_dir):
    nn = re.search(r"intrinsics_cam(\d+)\.yaml", os.path.basename(src_path)).group(1)
    d = yaml.safe_load(open(src_path))
    cm = d["camera_matrix"]["data"]
    fx, fy, cx, cy = cm[0][0], cm[1][1], cm[0][2], cm[1][2]
    dc = d["distortion_coefficients"]["data"]
    k1, k2, p1, p2 = dc[0], dc[1], dc[2], dc[3]
    w, h = d["image_width"], d["image_height"]
    text = TEMPLATE.format(focal=num(fx), k1=num(k1), k2=num(k2), p1=num(p1),
                           p2=num(p2), fx=num(fx), fy=num(fy), cx=num(cx),
                           cy=num(cy), w=w, h=h, nn=nn)
    out_path = os.path.join(out_dir, "camchain_%s.yaml" % nn)
    with open(out_path, "w") as f:
        f.write(text)
    return nn, out_path, (fx, fy, cx, cy), (k1, k2, p1, p2, dc[4] if len(dc) > 4 else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", default=DEFAULT_SRC)
    ap.add_argument("--out", default=DEFAULT_OUT)
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    srcs = sorted(glob.glob(os.path.join(args.src, "intrinsics_cam*.yaml")))
    if not srcs:
        raise SystemExit("no intrinsics_cam*.yaml found in %s" % args.src)

    print("converting %d files -> %s\n" % (len(srcs), args.out))
    written = []
    for s in srcs:
        nn, out_path, K, D = convert(s, args.out)
        written.append(out_path)
        print("  camchain_%s.yaml  focal=%.3f cx=%.2f cy=%.2f  "
              "D=[%.4f, %.4f, %.5f, %.5f]  (dropped k3=%s)"
              % (nn, K[0], K[2], K[3], D[0], D[1], D[2], D[3],
                 ("%.4f" % D[4]) if D[4] is not None else "none"))

    # Self-verify: every file must re-open as valid OpenCV FileStorage.
    print("\nverifying with cv2.FileStorage:")
    try:
        import cv2
        for p in written:
            fs = cv2.FileStorage(p, cv2.FILE_STORAGE_READ)
            K = fs.getNode("KMat").mat()
            Dn = fs.getNode("D")
            res = fs.getNode("resolution")
            ok = K is not None and K.shape == (3, 3)
            fs.release()
            print("  %s  KMat[0,0]=%.3f  ok=%s"
                  % (os.path.basename(p), K[0, 0] if ok else float("nan"), ok))
    except ImportError:
        print("  (cv2 unavailable; skipped read-back check)")


if __name__ == "__main__":
    main()
