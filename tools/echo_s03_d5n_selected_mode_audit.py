#!/usr/bin/env python3
"""Passive check of selected four-camera calibration after supervisor restart."""
import argparse
from collections import Counter
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, PointCloud2
from rtabmap_msgs.msg import RGBDImages

SIDES = ("front", "rear", "left", "right")
FX = 317.04311998
RAW = "/echo/cameras"
CANDIDATE = "/echo/s03/calibration/coupled_xy"


def check_info(info, xy, frame):
    cx, cy = xy
    expected = (FX, FX, cx, cy, FX, FX, cx, cy)
    observed = (info.k[0], info.k[4], info.k[2], info.k[5],
                info.p[0], info.p[5], info.p[2], info.p[6])
    return ((info.width, info.height) == (640, 480) and info.header.frame_id == frame
            and all(math.isclose(a,b,abs_tol=1e-4,rel_tol=0) for a,b in zip(observed, expected)))


def stamp(msg):
    return msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec


def cloud_projection(msg, xy, frame):
    """Infer the principal point from actual organized XYZ, independent of graph names."""
    import numpy as np
    if (msg.width, msg.height) != (640, 480) or msg.header.frame_id != frame:
        raise ValueError("unexpected cloud size or optical frame")
    offsets = {f.name: f.offset for f in msg.fields if f.datatype == 7 and f.count == 1}
    if not all(k in offsets for k in "xyz"):
        raise ValueError("missing FLOAT32 XYZ fields")
    dtype = ">f4" if msg.is_bigendian else "<f4"
    x, y, z = [np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                          offset=offsets[k], strides=(msg.row_step, msg.point_step))
               [::4, ::4].astype(np.float64) for k in "xyz"]
    v, u = np.mgrid[0:msg.height:4, 0:msg.width:4]
    valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z) & (z > 0.01)
    if np.count_nonzero(valid) < 1000:
        raise ValueError("too few valid XYZ samples")
    cx = u[valid] - FX*x[valid]/z[valid]
    cy = v[valid] - FX*y[valid]/z[valid]
    p99 = float(np.percentile(np.maximum(abs(cx-xy[0]), abs(cy-xy[1])), 99))
    return dict(samples=int(valid.sum()), cx_px=float(np.median(cx)),
                cy_px=float(np.median(cy)), p99_abs_error_px=p99,
                passed=p99 < 0.005)


def audit_clouds(node, xy, root):
    """Wake lazy converters one at a time; check routing and 3 distinct XYZ frames."""
    results = {}
    for side in SIDES:
        frame = f"echo_d455_{side}_depth_optical_frame"
        name = f"echo_d455_{side}_depth_to_cloud"
        expected = f"{root}/{side}/depth/camera_info"
        image = f"{RAW}/{side}/depth/image_raw"
        frames, failures = {}, []
        def receive(msg):
            if stamp(msg) in frames or len(frames) >= 3:
                return
            try:
                frames[stamp(msg)] = cloud_projection(msg, xy, frame)
            except Exception as exc:
                failures.append(str(exc))
        sub = node.create_subscription(PointCloud2, f"{RAW}/{side}/depth/points",
                                       receive, qos_profile_sensor_data)
        seen = []
        try:
            end = time.monotonic() + 15.0
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                if len(frames) >= 3:
                    try:
                        seen = sorted(t for t, _ in node.get_subscriber_names_and_types_by_node(name, "/"))
                    except Exception:
                        continue
                    if image in seen and expected in seen:
                        break
                    # Give DDS graph discovery time even if frames arrived first.
            infos = [t for t in seen if t.endswith("/camera_info")]
            passed = (len(frames) == 3 and not failures and image in seen and
                      infos == [expected] and all(x["passed"] for x in frames.values()))
            results[side] = dict(passed=passed, expected_camera_info=expected,
                                 subscriptions=seen, frames=list(frames.values()),
                                 errors=failures[:10])
        finally:
            node.destroy_subscription(sub)
    return results


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=("baseline", "coupled_xy"), required=True)
    p.add_argument("--seconds", type=float, default=20)
    p.add_argument("--clouds", action="store_true", help="Also verify all native XYZ outputs sequentially")
    args = p.parse_args()
    if not 5 <= args.seconds <= 120:
        p.error("seconds must be 5..120")
    xy = (319.5, 239.5) if args.mode == "coupled_xy" else (320., 240.)
    rclpy.init()
    node = Node("echo_s03_d5n_selected_mode_audit")
    counts, errors = Counter(), Counter()
    subs, expected_topics, arrivals, spreads = [], [], [], []

    def on_info(msg, topic, model, frame):
        counts[topic] += 1
        if not check_info(msg, model, frame):
            errors[topic] += 1

    def aggregate(msg):
        counts["aggregate_received"] += 1
        if len(msg.rgbd_images) != 4:
            errors["aggregate_camera_count"] += 1
            return
        stamps = []
        for side, item in zip(SIDES, msg.rgbd_images):
            frame = f"echo_d455_{side}_color_optical_frame"
            if not all(check_info(info, xy, frame) for info in (item.rgb_camera_info, item.depth_camera_info)):
                errors[f"{side}_embedded_model"] += 1
                return
            if item.rgb.header.frame_id != frame or item.depth.header.frame_id != frame:
                errors[f"{side}_image_frame"] += 1
                return
            if stamp(item.rgb) != stamp(item.depth):
                errors[f"{side}_image_stamp"] += 1
                return
            stamps.append(stamp(item.rgb))
        spreads.append((max(stamps)-min(stamps))/1e6)
        arrivals.append(time.monotonic())

    cloud_results = {}
    unexpected_corrected_publishers = []
    try:
        for root, model in [(RAW, (320.,240.))] + ([(CANDIDATE, xy)] if args.mode == "coupled_xy" else []):
            for side in SIDES:
                for sensor in ("color", "depth"):
                    topic = f"{root}/{side}/{sensor}/camera_info"
                    frame = f"echo_d455_{side}_{sensor}_optical_frame"
                    expected_topics.append(topic)
                    subs.append(node.create_subscription(CameraInfo, topic,
                        lambda msg, topic=topic, model=model, frame=frame: on_info(msg,topic,model,frame),
                        qos_profile_sensor_data))
        subs.append(node.create_subscription(RGBDImages,"/echo/s03/rgbd_images",aggregate,qos_profile_sensor_data))
        until = time.monotonic() + args.seconds
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
        if args.clouds:
            for sub in subs:
                node.destroy_subscription(sub)
            subs.clear()
            cloud_results = audit_clouds(node, xy, CANDIDATE if args.mode == "coupled_xy" else RAW)
        if args.mode == "baseline":
            unexpected_corrected_publishers = [f"{CANDIDATE}/{side}/{sensor}/camera_info"
                for side in SIDES for sensor in ("color", "depth")
                if node.count_publishers(f"{CANDIDATE}/{side}/{sensor}/camera_info")]
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    missing = [t for t in expected_topics if not counts[t]]
    hz = (len(arrivals)-1)/(arrivals[-1]-arrivals[0]) if len(arrivals)>1 else None
    passed = not missing and not errors and len(arrivals)>=10 and max(spreads,default=0)<=50.001
    passed = passed and not unexpected_corrected_publishers and (
        not args.clouds or (len(cloud_results) == 4 and all(r["passed"] for r in cloud_results.values())))
    print(json.dumps(dict(mode=args.mode, status="PASS" if passed else "FAIL", counts=dict(counts),
        errors=dict(errors),missing=missing,aggregate_valid=len(arrivals),arrival_hz=hz,
        max_spread_ms=max(spreads,default=None), native_clouds=cloud_results,
        unexpected_corrected_publishers=unexpected_corrected_publishers),indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
