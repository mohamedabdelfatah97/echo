#!/usr/bin/env python3
"""ECHO S02: offline, timestamp-checked LiDAR-to-front-D455 frustum crop.

Reads the existing dual-sensor ROS2 SQLite bag only, and writes a NEW PLY
and JSON report. Retains the LiDAR points whose projections lie inside the
recorded FRONT COLOR camera's pinhole field of view at scan time. This is
frustum gating, not depth-occlusion masking or sensor accuracy validation.

The global cropped LiDAR PLY uses the ROS `odom` frame. An exported RTAB-Map
D455 PLY may use a different optimized `map` frame and must be checked before
quantitative surface distance comparisons.
"""
from __future__ import annotations

import argparse
import json
import sqlite3
import sys
from pathlib import Path

import numpy as np

ROOT = Path.home() / "echo"
TOOLS = ROOT / "tools"
if not (TOOLS / "echo_s02_reconstruct_lidar.py").is_file():
    raise SystemExit("[STOP] Previously validated LiDAR reconstruction helper missing")
sys.path.insert(0, str(TOOLS))
from echo_s02_reconstruct_lidar import (  # noqa: E402
    stamp_s, to_matrix, interpolate_tf, voxel_filter, write_binary_ply
)

DEFAULT_BAG = ROOT / "diagnostics/s02/dual_cqFe5xh3/lidar_bag/lidar_bag_0.db3"
CAM_TOPIC = "/echo/cameras/front/color/camera_info"
LIDAR_TOPIC = "/front_3d_lidar/lidar_points"
CAM_FRAME = "echo_d455_front_color_optical_frame"
LIDAR_FRAME = "front_3d_lidar"
EDGES = (
    ("odom", "base_link"),
    ("base_link", "XT_32"),
    ("XT_32", LIDAR_FRAME),
    ("base_link", "nova_carter"),
    ("nova_carter", "RSD455"),
    ("RSD455", CAM_FRAME),
)


def read_tf(con, topic_ids, deserialize_message, TFMessage):
    observed = {edge: [] for edge in EDGES}
    for topic in ("/tf", "/tf_static"):
        if topic not in topic_ids:
            continue
        for (blob,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_ids[topic],),
        ):
            for tf in deserialize_message(blob, TFMessage).transforms:
                edge = (tf.header.frame_id.strip("/"), tf.child_frame_id.strip("/"))
                if edge not in observed:
                    continue
                t, q = tf.transform.translation, tf.transform.rotation
                observed[edge].append((
                    stamp_s(tf.header.stamp),
                    (t.x, t.y, t.z),
                    (q.x, q.y, q.z, q.w),
                ))
    for edge, rows in observed.items():
        if not rows:
            raise RuntimeError(f"Missing required transform: {edge}")
        rows.sort(key=lambda row: row[0])
    return observed


def fixed_transform(rows, label):
    first = rows[0]
    t0 = np.asarray(first[1], dtype=np.float64)
    q0 = np.asarray(first[2], dtype=np.float64)
    q0 = q0 / np.linalg.norm(q0)
    # Check ALL recorded sensor mount transforms are effectively fixed.
    for row in rows:
        t = np.asarray(row[1], dtype=np.float64)
        q = np.asarray(row[2], dtype=np.float64)
        q = q / np.linalg.norm(q)
        if np.linalg.norm(t - t0) > 0.001 or 1.0 - abs(np.dot(q, q0)) > 0.0001:
            raise RuntimeError(f"Non-static sensor/mount transform: {label}")
    return to_matrix(t0, q0)


def project_mask(xyz_lidar, lidar_to_cam, *, fx, fy, cx, cy, width, height,
                 near_m, far_m, lidar_min_m, lidar_max_m):
    """Camera pinhole model, ROI limited to RGB image boundaries."""
    xyz_lidar = np.asarray(xyz_lidar, dtype=np.float64)
    if not len(xyz_lidar):
        return np.zeros(0, dtype=bool), 0
    norms2 = np.einsum("ij,ij->i", xyz_lidar, xyz_lidar)
    range_mask = (np.isfinite(norms2) &
                  (norms2 >= lidar_min_m * lidar_min_m) &
                  (norms2 <= lidar_max_m * lidar_max_m))
    candidate = np.flatnonzero(range_mask)
    mask = np.zeros(len(xyz_lidar), dtype=bool)
    if not len(candidate):
        return mask, 0
    cam = xyz_lidar[candidate] @ lidar_to_cam[:3, :3].T + lidar_to_cam[:3, 3]
    z = cam[:, 2]
    valid = np.isfinite(cam).all(axis=1) & (z >= near_m) & (z <= far_m)
    valid_ids = np.flatnonzero(valid)
    if len(valid_ids):
        p = cam[valid_ids]
        u = fx * p[:, 0] / p[:, 2] + cx
        v = fy * p[:, 1] / p[:, 2] + cy
        inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        mask[candidate[valid_ids[inside]]] = True
    return mask, int(len(candidate))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-db", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--voxel", type=float, default=0.05)
    parser.add_argument("--near", type=float, default=0.5, help="D455 camera optical depth lower bound, m")
    parser.add_argument("--far", type=float, default=10.0, help="D455 camera optical depth upper bound, m")
    parser.add_argument("--lidar-min", type=float, default=0.5)
    parser.add_argument("--lidar-max", type=float, default=10.0)
    parser.add_argument("--max-tf-gap", type=float, default=0.25)
    parser.add_argument("--max-camera-dt", type=float, default=0.1,
                        help="Max difference to a camera_info stamp, seconds")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if (args.stride < 1 or args.voxel <= 0 or args.near < 0 or args.far <= args.near
            or args.lidar_min < 0 or args.lidar_max <= args.lidar_min
            or args.max_tf_gap <= 0 or args.max_camera_dt <= 0):
        parser.error("Invalid sampling, distance, voxel or timestamp parameters")
    bag = args.bag_db.expanduser().resolve()
    if not bag.is_file():
        parser.error(f"Missing bag file: {bag}")
    output = (args.output or bag.parent.parent /
              f"lidar_in_d455_color_fov_odom_{args.far:g}m_{args.voxel:g}m_stride{args.stride}.ply")
    output = output.expanduser().resolve()
    report = output.with_suffix(".json")
    if not output.parent.is_dir() or output.exists() or report.exists():
        parser.error("Output parent must exist; output PLY and JSON must not already exist")

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CameraInfo, PointCloud2
    from tf2_msgs.msg import TFMessage
    from sensor_msgs_py import point_cloud2

    print("=== ECHO S02: LIDAR CROP TO D455 RGB CAMERA FOV ===", flush=True)
    print("Input (read only):", bag, flush=True)
    print("Output:", output, flush=True)
    con = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
    try:
        con.execute("PRAGMA query_only=ON")
        ids = dict(con.execute("SELECT name, id FROM topics"))
        for topic in (CAM_TOPIC, LIDAR_TOPIC):
            if topic not in ids:
                raise RuntimeError(f"Required recorded topic missing: {topic}")
        tf = read_tf(con, ids, deserialize_message, TFMessage)
        trajectory = tf[("odom", "base_link")]
        odom_ts = np.asarray([x[0] for x in trajectory], dtype=np.float64)
        odom_t = np.asarray([x[1] for x in trajectory], dtype=np.float64)
        odom_q = np.asarray([x[2] for x in trajectory], dtype=np.float64)
        base_lidar = np.eye(4)
        for edge in (("base_link", "XT_32"), ("XT_32", LIDAR_FRAME)):
            base_lidar = base_lidar @ fixed_transform(tf[edge], str(edge))
        base_cam = np.eye(4)
        for edge in (("base_link", "nova_carter"), ("nova_carter", "RSD455"),
                     ("RSD455", CAM_FRAME)):
            base_cam = base_cam @ fixed_transform(tf[edge], str(edge))
        lidar_to_cam = np.linalg.inv(base_cam) @ base_lidar
        print("[ECHO] LiDAR-to-camera optical translation (m):",
              np.round(lidar_to_cam[:3, 3], 4).tolist(), flush=True)

        camera_stamps = []
        first_camera = None
        for (blob,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (ids[CAM_TOPIC],)
        ):
            info = deserialize_message(blob, CameraInfo)
            if info.header.frame_id.strip("/") != CAM_FRAME:
                raise RuntimeError(f"Unexpected color CameraInfo frame: {info.header.frame_id!r}")
            if first_camera is None:
                first_camera = info
            else:
                if (info.width != first_camera.width or info.height != first_camera.height
                        or not np.allclose(info.k, first_camera.k, atol=1e-7)):
                    raise RuntimeError("Camera calibration changed within the recording")
            camera_stamps.append(stamp_s(info.header.stamp))
        if first_camera is None:
            raise RuntimeError("No CameraInfo observations")
        if first_camera.distortion_model not in ("", "plumb_bob", "rational_polynomial"):
            raise RuntimeError(f"Unsupported camera distortion model: {first_camera.distortion_model}")
        if first_camera.d and max(abs(v) for v in first_camera.d) > 1e-6:
            raise RuntimeError("Nonzero color lens distortion; pinhole crop needs distortion correction")
        fx, fy = float(first_camera.k[0]), float(first_camera.k[4])
        cx, cy = float(first_camera.k[2]), float(first_camera.k[5])
        if min(fx, fy, first_camera.width, first_camera.height) <= 0:
            raise RuntimeError("Invalid CameraInfo intrinsic parameters")
        camera_stamps = np.asarray(sorted(camera_stamps), dtype=np.float64)
        print(f"[ECHO] Front COLOR CameraInfo: {first_camera.width}x{first_camera.height}, "
              f"fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}", flush=True)
        print(f"[ECHO] CameraInfo messages: {len(camera_stamps)} | "
              f"odom TF samples: {len(odom_ts)}", flush=True)

        seen_scans = valid_scans = skipped_tf = skipped_cam = 0
        cropped_xyz = []
        all_ranged = 0
        all_frustum = 0
        deltas = []
        for (blob,) in con.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (ids[LIDAR_TOPIC],)
        ):
            seen_scans += 1
            if (seen_scans - 1) % args.stride:
                continue
            scan = deserialize_message(blob, PointCloud2)
            if scan.header.frame_id.strip("/") != LIDAR_FRAME:
                raise RuntimeError(f"Unexpected point cloud frame: {scan.header.frame_id}")
            scan_stamp = stamp_s(scan.header.stamp)
            i = np.searchsorted(camera_stamps, scan_stamp)
            nearest = min((abs(camera_stamps[j] - scan_stamp)
                           for j in (i - 1, i) if 0 <= j < len(camera_stamps)),
                          default=float("inf"))
            if nearest > args.max_camera_dt:
                skipped_cam += 1
                continue
            odom_base = interpolate_tf((odom_ts, odom_t, odom_q),
                                       scan_stamp, args.max_tf_gap)
            if odom_base is None:
                skipped_tf += 1
                continue
            xyz = np.asarray(point_cloud2.read_points_numpy(
                scan, field_names=("x", "y", "z"), skip_nans=True)).reshape(-1, 3)
            mask, ranged = project_mask(
                xyz, lidar_to_cam, fx=fx, fy=fy, cx=cx, cy=cy,
                width=first_camera.width, height=first_camera.height,
                near_m=args.near, far_m=args.far,
                lidar_min_m=args.lidar_min, lidar_max_m=args.lidar_max,
            )
            all_ranged += ranged
            all_frustum += int(mask.sum())
            deltas.append(nearest)
            valid_scans += 1
            if mask.any():
                T = odom_base @ base_lidar
                world = xyz[mask] @ T[:3, :3].T + T[:3, 3]
                cropped_xyz.append(voxel_filter(world, args.voxel))
            if valid_scans % 20 == 0:
                print(f"[ECHO] {valid_scans} scans; {all_frustum:,} points in D455 FOV",
                      flush=True)
            if sum(len(x) for x in cropped_xyz) > 800_000:
                cropped_xyz = [voxel_filter(np.concatenate(cropped_xyz, axis=0), args.voxel)]
        if valid_scans < 3 or not cropped_xyz:
            raise RuntimeError(f"Insufficient camera/FOV-matched data: {valid_scans} scans; "
                               f"{all_frustum} retained points")
        points = voxel_filter(np.concatenate(cropped_xyz, axis=0), args.voxel)
        camera_dt = np.asarray(deltas)
        report_data = {
            "source_bag": str(bag), "output_ply": str(output),
            "frame": "odom", "lidar_frame": LIDAR_FRAME, "camera_frame": CAM_FRAME,
            "camera_topic": CAM_TOPIC, "camera_width_px": first_camera.width,
            "camera_height_px": first_camera.height,
            "camera_K": [float(x) for x in first_camera.k],
            "camera_distortion_model": first_camera.distortion_model,
            "camera_distortion": list(first_camera.d),
            "lidar_to_cam_4x4": lidar_to_cam.tolist(),
            "selected_scan_count": valid_scans, "total_scan_count": seen_scans,
            "skipped_tf": skipped_tf, "skipped_cam_time": skipped_cam,
            "camera_dt_median_s": float(np.median(camera_dt)),
            "camera_dt_p95_s": float(np.percentile(camera_dt, 95)),
            "points_in_lidar_range_before_frustum": all_ranged,
            "points_in_camera_frustum_before_voxel": all_frustum,
            "points_after_voxel": int(len(points)),
            "camera_near_depth_m": args.near, "camera_far_depth_m": args.far,
            "lidar_range_m": [args.lidar_min, args.lidar_max],
            "voxel_m": args.voxel, "stride": args.stride,
            "xyz_min_odom_m": points.min(axis=0).tolist(),
            "xyz_max_odom_m": points.max(axis=0).tolist(),
            "limitations": [
                "FOV projection is not a depth-occlusion or same-surface mask",
                "CameraInfo timing is checked; RGB/depth images are not in this LiDAR bag",
                "D455 dense export may be in RTAB-Map optimized map coordinates",
                "Simulated D455 depth is idealized; this is not hardware performance validation",
            ],
        }
        # Only write after all validation is complete, using exclusive mode.
        with output.open("xb") as file_handle:
            file_handle.write(("ply\nformat binary_little_endian 1.0\n"
                               "comment ECHO LiDAR in front D455 COLOR frustum, odom frame\n"
                               f"element vertex {len(points)}\n"
                               "property float x\nproperty float y\nproperty float z\n"
                               "end_header\n").encode("ascii"))
            file_handle.write(np.asarray(points, dtype="<f4", order="C").tobytes())
        with report.open("x") as f:
            json.dump(report_data, f, indent=2)
            f.write("\n")
        print("\n=== S02 CALIBRATED FRUSTUM CROP RESULT ===")
        print(f"LiDAR scans used: {valid_scans} / {seen_scans}; "
              f"skipped TF={skipped_tf}; camera timing={skipped_cam}")
        print(f"CameraInfo timing difference: median={np.median(camera_dt)*1e3:.1f} ms, "
              f"P95={np.percentile(camera_dt,95)*1e3:.1f} ms")
        print(f"In-range LiDAR points: {all_ranged:,}")
        print(f"Inside D455 camera FOV: {all_frustum:,} "
              f"({all_frustum / max(all_ranged,1)*100:.2f}% of LiDAR in-range points)")
        print(f"Cloud points after 5 cm voxelization: {len(points):,}")
        print("Output:", output)
        print("Metadata:", report)
        print("[PASS] Cropped LiDAR generated; input recordings unchanged")
        print("[NOTE] Frustum agreement does not imply occlusion-matched surface accuracy")
    finally:
        con.close()


if __name__ == "__main__":
    main()
