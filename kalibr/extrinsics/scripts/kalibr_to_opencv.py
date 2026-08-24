#!/usr/bin/env python3
"""
Convert a Kalibr camchain.yaml into:
  (a) per-camera OpenCV-style intrinsics yaml — same schema as your
      intrinsics_cam<ID>.yaml (camera_matrix / distortion_coefficients /
      plumb_bob), so it is drop-in for the camera2camera pipeline; and
  (b) one extrinsics yaml holding each adjacent transform in 4x4 + quaternion
      + yaw/pitch/roll + baseline form.

Kalibr `radtan` has 4 coeffs [k1,k2,p1,p2]; OpenCV `plumb_bob` has 5
[k1,k2,p1,p2,k3].  Kalibr did not estimate k3, so it is written as 0.0
(documented in each file's `note`).

Usage:
  python3 kalibr_to_opencv.py --camchain 66_64_65-camchain.yaml
"""
import argparse
import os
import re
import numpy as np
import yaml


def cam_id(topic):
    m = re.search(r"cam(\d+)", topic or "")
    return m.group(1) if m else None


def rot_to_quat(R):  # [x, y, z, w]
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, 0.25*s])
    i = int(np.argmax([R[0,0], R[1,1], R[2,2]]))
    if i == 0:
        s = np.sqrt(1.0+R[0,0]-R[1,1]-R[2,2])*2
        return np.array([0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
    if i == 1:
        s = np.sqrt(1.0-R[0,0]+R[1,1]-R[2,2])*2
        return np.array([(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
    s = np.sqrt(1.0-R[0,0]-R[1,1]+R[2,2])*2
    return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s])


def rot_to_euler_deg(R):
    """Per camera-optical-axis angles (deg): Y=pan/yaw, X=tilt/pitch, Z=roll."""
    about_y = np.degrees(np.arcsin(float(np.clip(-R[2,0], -1, 1))))
    if abs(R[2,0]) < 0.99999:
        about_x = np.degrees(np.arctan2(R[2,1], R[2,2]))
        about_z = np.degrees(np.arctan2(R[1,0], R[0,0]))
    else:
        about_x = np.degrees(np.arctan2(-R[1,2], R[1,1])); about_z = 0.0
    return {"yaw_pan_about_Y_deg": round(float(about_y), 4),
            "pitch_tilt_about_X_deg": round(float(about_x), 4),
            "roll_about_Z_deg": round(float(about_z), 4)}


def main():
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # scripts/ -> extrinsics/
    ap = argparse.ArgumentParser()
    ap.add_argument("--camchain", default=os.path.join(here, "66_64_65-camchain.yaml"))
    ap.add_argument("--out-dir", default=None)
    ap.add_argument("--no-matrices", action="store_true",
                    help="emit flat scalars (fx,fy,cx,cy + translation/rotation) instead "
                         "of the 3x3 K camera_matrix, the R matrix, and the 4x4 transform")
    args = ap.parse_args()
    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.camchain))
    stem = os.path.basename(args.camchain).replace("-camchain.yaml", "")

    with open(args.camchain) as f:
        doc = yaml.safe_load(f)
    keys = sorted(doc.keys())                       # cam0, cam1, ...

    written = []
    # (a) per-camera OpenCV intrinsics
    for k in keys:
        c = doc[k]
        fx, fy, cx, cy = c["intrinsics"]
        d = list(c["distortion_coeffs"])            # [k1,k2,p1,p2]
        d5 = d + [0.0] * (5 - len(d))               # -> plumb_bob, k3=0
        W, H = c["resolution"]
        cid = cam_id(c.get("rostopic"))
        note = (f"converted from kalibr {os.path.basename(args.camchain)} "
                f"({k}, radtan-4 -> plumb_bob with k3=0)")
        if args.no_matrices:
            intr = {"fx": fx, "fy": fy, "cx": cx, "cy": cy,
                    "distortion_model": "plumb_bob",
                    "distortion_coefficients": d5,
                    "image_width": int(W), "image_height": int(H),
                    "note": note}
        else:
            intr = {
                "image_width": int(W), "image_height": int(H),
                "camera_matrix": {"rows": 3, "cols": 3,
                                  "data": [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]},
                "distortion_model": "plumb_bob",
                "distortion_coefficients": {"rows": 1, "cols": 5, "data": d5},
                "note": note}
        path = os.path.join(out_dir, f"kalibr_intrinsics_cam{cid}.yaml")
        with open(path, "w") as f:
            yaml.safe_dump(intr, f, default_flow_style=False, sort_keys=not args.no_matrices)
        written.append(path)

    # (b) extrinsics chain
    chain = [cam_id(doc[k].get("rostopic")) for k in keys]
    transforms = []
    for k in keys:
        c = doc[k]
        if "T_cn_cnm1" not in c:
            continue
        idx = keys.index(k)
        parent, child = chain[idx - 1], chain[idx]
        T = np.array(c["T_cn_cnm1"]); R = T[:3, :3]; t = T[:3, 3]
        tr = {
            "parent": f"cam{parent}", "child": f"cam{child}",
            "frame_convention": "rotation + translation map a point in PARENT frame "
                                "into CHILD frame (kalibr T_cn_cnm1)",
            "translation_m": t.tolist(),
            "rotation": {"quaternion_xyzw": rot_to_quat(R).tolist(),
                         "euler": rot_to_euler_deg(R)},
            "baseline_m": round(float(np.linalg.norm(t)), 5),
        }
        if not args.no_matrices:
            tr["T_child_parent"] = T.tolist()
        transforms.append(tr)
    ext = {"chain": [f"cam{c}" for c in chain],
           "source": os.path.basename(args.camchain),
           "transforms": transforms}
    epath = os.path.join(out_dir, f"{stem}-extrinsics.yaml")
    with open(epath, "w") as f:
        yaml.safe_dump(ext, f, default_flow_style=False, sort_keys=False)
    written.append(epath)

    print("wrote:")
    for p in written:
        print("  " + p)
    print("\n--- extrinsics summary ---")
    for tr in transforms:
        e = tr["rotation"]["euler"]
        print(f"  {tr['parent']} -> {tr['child']}: pan(yaw)={e['yaw_pan_about_Y_deg']:+.2f} "
              f"pitch={e['pitch_tilt_about_X_deg']:+.2f} roll={e['roll_about_Z_deg']:+.2f} deg | "
              f"baseline={tr['baseline_m']*100:.1f} cm")


if __name__ == "__main__":
    main()
