#!/usr/bin/env python3
"""Record one RTSP stream to Matroska segments with a per-frame timing sidecar."""
import argparse
import os
import re
import signal
import sys

import gi
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--codec", default="h265", choices=("h264", "h265"))
    parser.add_argument("--out-pattern", required=True)
    parser.add_argument("--sidecar", required=True)
    parser.add_argument("--seg-sec", type=int, default=3600)
    parser.add_argument("--max-files", type=int, default=0)
    parser.add_argument("--latency", type=int, default=100)
    parser.add_argument("--duration", type=int, default=0)
    args = parser.parse_args()
    if args.seg_sec <= 0 or args.max_files < 0 or args.latency < 0 or args.duration < 0:
        parser.error("segment length must be positive; other numeric options must be non-negative")
    uri = os.environ.get("KALIBR_RTSP_URI")
    if not uri:
        parser.error("KALIBR_RTSP_URI environment variable is required")
    if any(char in uri for char in '"\n\r'):
        parser.error("RTSP URI contains unsupported characters")

    Gst.init(None)
    depay, parse = (("rtph265depay", "h265parse") if args.codec == "h265"
                    else ("rtph264depay", "h264parse"))
    desc = (
        'rtspsrc name=src latency={} protocols=tcp ntp-sync=false '
        'short-header=true ! {} ! {} config-interval=-1 ! tee name=t '
        't. ! queue max-size-buffers=0 max-size-time=0 '
        '! splitmuxsink name=segments max-size-time={} '
        'muxer-factory=matroskamux async-finalize=true send-keyframe-requests=true '
        't. ! queue max-size-buffers=0 max-size-time=0 '
        '! appsink name=ts emit-signals=true sync=false drop=false max-buffers=0'
    ).format(args.latency, depay, parse, args.seg_sec * Gst.SECOND)

    try:
        pipeline = Gst.parse_launch(desc)
    except GLib.Error as exc:
        print("ERROR: cannot create GStreamer pipeline: {}".format(exc), file=sys.stderr)
        return 1
    source = pipeline.get_by_name("src")
    source.set_property("location", uri)
    sink = pipeline.get_by_name("segments")
    sink.set_property("location", args.out_pattern)
    if args.max_files:
        sink.set_property("max-files", args.max_files)

    clock = Gst.SystemClock.obtain()
    clock.set_property("clock-type", Gst.ClockType.REALTIME)
    pipeline.use_clock(clock)
    sidecar = open(args.sidecar, "w", buffering=1)
    sidecar.write("frame_idx,keyframe,pts_ns,clk_unix_ns,recv_unix_ns\n")
    state = {"count": 0, "base": None, "started": False, "error": False}

    def on_sample(appsink):
        import time
        received = time.time_ns()
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK
        buffer = sample.get_buffer()
        keyframe = not bool(buffer.get_flags() & Gst.BufferFlags.DELTA_UNIT)
        if not state["started"]:
            if not keyframe:
                return Gst.FlowReturn.OK
            state["started"] = True
        if state["base"] is None:
            state["base"] = pipeline.get_base_time()
        pts = buffer.pts
        clock_ns = (state["base"] + pts
                    if pts != Gst.CLOCK_TIME_NONE and
                    state["base"] != Gst.CLOCK_TIME_NONE else "")
        sidecar.write("{},{},{},{},{}\n".format(
            state["count"], int(keyframe), pts, clock_ns, received))
        state["count"] += 1
        return Gst.FlowReturn.OK

    pipeline.get_by_name("ts").connect("new-sample", on_sample)
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def on_message(_bus, message):
        if message.type == Gst.MessageType.EOS:
            loop.quit()
        elif message.type == Gst.MessageType.ERROR:
            error, debug = message.parse_error()
            safe_authority = re.sub(r"(?<=rtsp://).*?@", "[redacted]@", uri)
            detail = "{} :: {}".format(error, debug).replace(uri, safe_authority)
            if "@" in uri:
                credentials = uri.split("://", 1)[1].split("@", 1)[0]
                detail = detail.replace(credentials, "[redacted]")
            print("ERROR: {}".format(detail), file=sys.stderr)
            state["error"] = True
            loop.quit()

    bus.connect("message", on_message)
    stopping = {"value": False}

    def request_stop(*_args):
        if stopping["value"]:
            loop.quit()
        else:
            stopping["value"] = True
            pipeline.send_event(Gst.Event.new_eos())
            GLib.timeout_add_seconds(30, lambda: (loop.quit(), False)[1])
        return False

    GLib.unix_signal_add(GLib.PRIORITY_HIGH, signal.SIGINT, request_stop)
    GLib.unix_signal_add(GLib.PRIORITY_HIGH, signal.SIGTERM, request_stop)
    if pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
        print("ERROR: GStreamer pipeline failed to start", file=sys.stderr)
        pipeline.set_state(Gst.State.NULL)
        sidecar.close()
        return 1
    if args.duration:
        GLib.timeout_add_seconds(args.duration, request_stop)
    try:
        loop.run()
    finally:
        pipeline.set_state(Gst.State.NULL)
        sidecar.close()
    return 1 if state["error"] else 0


if __name__ == "__main__":
    sys.exit(main())
