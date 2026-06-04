#!/usr/bin/env python3
"""
Tabulate each camera's yaw / roll / pitch RELATIVE to a reference camera
(default cam64 = 0,0,0), derived from a kalibr camchain.yaml.

The camchain stores per-link T_cn_cnm1 (cam_{n-1} -> cam_n). We compose those
into each camera's rotation, then re-express it in the reference camera's frame
(R_{cam_i <- ref}) and read off Euler angles in the camera-optical convention:
  yaw  = rotation about Y (vertical)  -> horizontal pan
  pitch= rotation about X (right)     -> vertical tilt
  roll = rotation about Z (optical)
Prints a markdown table (+ optional --out file).
"""
import argparse
import os
import re
import numpy as np
import yaml


def cam_id(t):
    m = re.search(r"cam(\d+)", t or "")
    return m.group(1) if m else None


def euler_ypr(R):
    """Return (yaw_about_Y, pitch_about_X, roll_about_Z) in degrees.

    Uses a Y-PRIMARY decomposition R = Ry(yaw)*Rx(pitch)*Rz(roll), so the pan
    (about Y) is the first axis and may reach +-90 deg+ without gimbal lock;
    the singularity is on the middle axis (pitch), which is small for this rig.
    """
    pitch = np.degrees(np.arcsin(float(np.clip(-R[1, 2], -1, 1))))   # about X
    if abs(R[1, 2]) < 0.99999:                                       # not gimbal-locked
        yaw = np.degrees(np.arctan2(R[0, 2], R[2, 2]))               # about Y
        roll = np.degrees(np.arctan2(R[1, 0], R[1, 1]))              # about Z
    else:
        yaw = np.degrees(np.arctan2(-R[2, 0], R[0, 0])); roll = 0.0
    return yaw, pitch, roll


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # scripts/ -> extrinsics/
    ap = argparse.ArgumentParser()
    ap.add_argument("--camchain", default=os.path.join(here, "68_66_64_65_67-camchain.yaml"))
    ap.add_argument("--ref", default="64", help="reference camera id treated as 0")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    doc = yaml.safe_load(open(args.camchain))
    keys = sorted(doc.keys())                       # cam0, cam1, ...
    ids = [cam_id(doc[k]["rostopic"]) for k in keys]

    # R_abs[i] = rotation mapping cam0 frame -> cam_i frame (compose T_cn_cnm1)
    R_abs = [np.eye(3)]
    for i in range(1, len(keys)):
        R_link = np.array(doc[keys[i]]["T_cn_cnm1"])[:3, :3]   # cam_{i-1} -> cam_i
        R_abs.append(R_link @ R_abs[-1])

    if args.ref not in ids:
        raise SystemExit(f"ref cam{args.ref} not in chain {ids}")
    R_ref = R_abs[ids.index(args.ref)]

    rows = []
    for i, cid in enumerate(ids):
        R_rel = R_abs[i] @ R_ref.T                  # cam_ref frame -> cam_i frame
        yaw, pitch, roll = euler_ypr(R_rel)
        rows.append((cid, yaw, roll, pitch))

    lines = [f"Yaw/Roll/Pitch relative to cam{args.ref} (= 0), from {os.path.basename(args.camchain)}",
             "",
             "| Camera | Yaw (pan, about Y) ° | Roll (about Z) ° | Pitch (tilt, about X) ° |",
             "|---|---:|---:|---:|"]
    for cid, yaw, roll, pitch in rows:
        tag = "  *(reference)*" if cid == args.ref else ""
        lines.append(f"| **cam{cid}**{tag} | {yaw:+.2f} | {roll:+.2f} | {pitch:+.2f} |")
    md = "\n".join(lines)
    print(md)
    if args.out:
        with open(args.out, "w") as f:
            f.write(md + "\n")
        print(f"\n[written] {args.out}")


if __name__ == "__main__":
    main()
