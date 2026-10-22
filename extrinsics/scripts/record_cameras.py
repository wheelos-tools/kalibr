#!/usr/bin/env python3
"""Record the five Kalibr rig cameras concurrently into Matroska segments."""
import argparse
import datetime
import ipaddress
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from urllib.parse import quote

CHAIN = ("cam68", "cam66", "cam64", "cam65", "cam67")
SCRIPT_DIR = Path(__file__).resolve().parent
LOCAL_CONFIG = SCRIPT_DIR.parent / "cameras.local.json"
DEFAULT_CONFIG = (LOCAL_CONFIG if LOCAL_CONFIG.exists()
                  else SCRIPT_DIR.parent / "cameras.example.json")


def read_cameras(config_path):
    try:
        with open(config_path, encoding="utf-8") as stream:
            data = json.load(stream)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError("cannot read camera config {}: {}".format(config_path, exc))

    if not isinstance(data, dict):
        raise ValueError("config root must be a JSON object")
    cameras = data.get("cameras")
    if not isinstance(cameras, list):
        raise ValueError("config must contain a 'cameras' list")

    found = {}
    for camera in cameras:
        if not isinstance(camera, dict):
            raise ValueError("each camera entry must be an object")
        camera_id = camera.get("id")
        if camera_id not in CHAIN:
            raise ValueError("unsupported camera id: {!r}".format(camera_id))
        if camera_id in found:
            raise ValueError("duplicate camera id: {}".format(camera_id))
        host = camera.get("host")
        if not isinstance(host, str) or not host or any(c in host for c in '/"\\\n'):
            raise ValueError("invalid host for {}".format(camera_id))
        try:
            ipaddress.ip_address(host)
        except ValueError:
            if not re.fullmatch(r"[A-Za-z0-9.-]+", host):
                raise ValueError("invalid host for {}".format(camera_id))
        codec = camera.get("codec", "h265")
        if codec not in ("h264", "h265"):
            raise ValueError("unsupported codec for {}: {}".format(camera_id, codec))
        port = camera.get("port", 554)
        if not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("invalid port for {}".format(camera_id))
        path = camera.get("path", "/live")
        if not isinstance(path, str) or not path.startswith("/") or '"' in path:
            raise ValueError("path must be an absolute RTSP path for {}".format(camera_id))
        user_var = camera.get("username_env")
        password_var = camera.get("password_env")
        if ((user_var is not None and not isinstance(user_var, str)) or
                (password_var is not None and not isinstance(password_var, str))):
            raise ValueError("credential environment variable names must be strings")
        found[camera_id] = dict(camera_id=camera_id, host=host, port=port,
                                path=path, codec=codec,
                                username_env=user_var, password_env=password_var)

    missing = [camera_id for camera_id in CHAIN if camera_id not in found]
    if missing:
        raise ValueError("config is missing cameras: {}".format(", ".join(missing)))
    return [found[camera_id] for camera_id in CHAIN]


def camera_uri(camera):
    user_var = camera["username_env"]
    password_var = camera["password_env"]
    if bool(user_var) != bool(password_var):
        raise ValueError("{} must configure both credential environment variable names"
                         .format(camera["camera_id"]))
    auth = ""
    if user_var:
        user = os.environ.get(user_var)
        password = os.environ.get(password_var)
        if user is None or password is None:
            raise ValueError("set both {} and {} before recording"
                             .format(user_var, password_var))
        auth = "{}:{}@".format(quote(user, safe=""), quote(password, safe=""))
    return "rtsp://{}{}:{}{}".format(
        auth, camera["host"], camera["port"], quote(camera["path"], safe="/"))


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record the five Kalibr cameras concurrently; credentials are read from environment variables.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help="JSON camera config (default: %(default)s)")
    parser.add_argument("--out", default=str(SCRIPT_DIR.parent / "recordings"),
                        help="recording root directory")
    parser.add_argument("-d", "--duration", type=int, default=0,
                        help="total recording seconds; 0 runs until Ctrl-C")
    parser.add_argument("-l", "--length", type=int, default=3600,
                        help="segment length in seconds (default: one-hour segments)")
    parser.add_argument("-n", "--count", type=int, default=0,
                        help="maximum segments per camera; 0 is unlimited")
    parser.add_argument("-c", "--cams", default="",
                        help="comma-separated camera IDs or IP last octets")
    parser.add_argument("--latency", type=int, default=100,
                        help="GStreamer RTSP jitter buffer latency in ms")
    args = parser.parse_args()
    if args.duration < 0 or args.count < 0 or args.length <= 0 or args.latency < 0:
        parser.error("duration/count/latency must be non-negative and length must be positive")
    return args


def main():
    args = parse_args()
    try:
        cameras = read_cameras(args.config)
    except ValueError as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2

    selected = {token.strip() for token in args.cams.split(",") if token.strip()}
    unknown = selected - ({c["camera_id"] for c in cameras} |
                         {c["host"].rsplit(".", 1)[-1] for c in cameras})
    if unknown:
        print("ERROR: unknown camera filter(s): {}".format(", ".join(sorted(unknown))),
              file=sys.stderr)
        return 2
    cameras = [c for c in cameras if not selected or
               c["camera_id"] in selected or c["host"].rsplit(".", 1)[-1] in selected]
    if not cameras:
        print("ERROR: no cameras selected", file=sys.stderr)
        return 2
    try:
        for camera in cameras:
            camera["uri"] = camera_uri(camera)
    except ValueError as exc:
        print("ERROR: {}".format(exc), file=sys.stderr)
        return 2

    run_id = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.out).expanduser() / run_id
    try:
        run_dir.mkdir(parents=True, exist_ok=False)
    except OSError as exc:
        print("ERROR: cannot create recording directory {}: {}".format(run_dir, exc),
              file=sys.stderr)
        return 2

    print("[record] run directory: {}".format(run_dir))
    print("[record] cameras: {}".format(", ".join(c["camera_id"] for c in cameras)))
    print("[record] segment={}s duration={}s (0=until Ctrl-C)"
          .format(args.length, args.duration))

    children = []
    stopping = False
    failure_reported = False

    def stop_children(_signum=None, _frame=None):
        nonlocal stopping
        if stopping:
            return
        stopping = True
        print("\n[record] requesting graceful stop for {} recorder(s)"
              .format(len(children)), flush=True)
        for proc, _log in children:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)

    previous_int = signal.signal(signal.SIGINT, stop_children)
    previous_term = signal.signal(signal.SIGTERM, stop_children)
    try:
        for camera in cameras:
            if stopping:
                break
            cam_id = camera["camera_id"]
            out_dir = run_dir / cam_id
            out_dir.mkdir()
            prefix = "{}_{}".format(cam_id, camera["host"])
            out_pattern = out_dir / (prefix + "_%05d.mkv")
            sidecar = out_dir / (prefix + ".frames.csv")
            log_path = out_dir / (prefix + ".log")
            env = os.environ.copy()
            env["KALIBR_RTSP_URI"] = camera["uri"]
            command = [
                sys.executable, str(SCRIPT_DIR / "record_cam_synced.py"),
                "--codec", camera["codec"], "--out-pattern", str(out_pattern),
                "--sidecar", str(sidecar), "--seg-sec", str(args.length),
                "--max-files", str(args.count), "--latency", str(args.latency),
                "--duration", str(args.duration),
            ]
            log = open(log_path, "w", encoding="utf-8")
            try:
                proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                        env=env)
            except OSError:
                log.close()
                raise
            children.append((proc, log))
            print("[record] {} {} codec={} -> {}"
                  .format(cam_id, camera["host"], camera["codec"], out_pattern),
                  flush=True)

        while True:
            statuses = [proc.poll() for proc, _log in children]
            if all(status is not None for status in statuses):
                failures = [(proc.pid, proc.returncode)
                            for proc, _log in children if proc.returncode]
                if failures:
                    print("[record] ERROR: recorder process failure(s): {}"
                          .format(failures), file=sys.stderr)
                    return 1
                break
            failed = [(proc, status) for (proc, _log), status in zip(children, statuses)
                      if status not in (None, 0)]
            if failed and not failure_reported:
                failure_reported = True
                print("[record] ERROR: a recorder failed; stopping remaining cameras",
                      file=sys.stderr)
                stop_children()
            time.sleep(0.2)
    except OSError as exc:
        print("[record] ERROR: {}".format(exc), file=sys.stderr)
        stop_children()
        return 1
    finally:
        if any(proc.poll() is None for proc, _log in children):
            stop_children()
            for proc, _log in children:
                try:
                    proc.wait(timeout=35)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
        for _proc, log in children:
            log.close()
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)
    print("[record] finished: {}".format(run_dir))
    return 0


if __name__ == "__main__":
    sys.exit(main())
