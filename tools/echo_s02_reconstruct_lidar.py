#!/usr/bin/env python3
"""ECHO S02: offline LiDAR cloud in odom from a ROS 2 SQLite bag.

Does not run Isaac Sim, publish ROS commands, or change a source database.
Dependencies: ROS 2 Jazzy's rclpy, tf2_msgs, sensor_msgs, sensor_msgs_py; numpy.
"""
from __future__ import annotations

import argparse
import json
import math
import sqlite3
import struct
from pathlib import Path

import numpy as np

DEFAULT_BAG = Path.home() / "echo/diagnostics/s02/dual_cqFe5xh3/lidar_bag"
EDGE_ODOM = ("odom", "base_link")
EDGE_LIDAR1 = ("base_link", "XT_32")
EDGE_LIDAR2 = ("XT_32", "front_3d_lidar")


def stamp_s(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def quat_matrix(q):
    q = np.asarray(q, dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm < 1e-8:
        raise ValueError("Zero quaternion in recorded TF")
    x, y, z, w = q / norm
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def to_matrix(translation, quaternion):
    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = quat_matrix(quaternion)
    mat[:3, 3] = translation
    return mat


def slerp(a, b, fraction):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    dot = float(np.dot(a, b))
    if dot < 0:
        b = -b
        dot = -dot
    dot = np.clip(dot, -1., 1.)
    if dot > 0.9995:
        c = a + fraction * (b - a)
        return c / np.linalg.norm(c)
    theta = math.acos(dot)
    return (math.sin((1-fraction)*theta) * a + math.sin(fraction*theta) * b) / math.sin(theta)


def voxel_filter(points, voxel_m):
    """Preserve one representative point per voxel; sensor comparison uses same rule."""
    if not len(points):
        return np.empty((0, 3), dtype=np.float32)
    keys = np.floor(points / voxel_m).astype(np.int32)
    # Fixed-length key rows permit efficient 1D unique without Python sets.
    packed = np.ascontiguousarray(keys).view(np.dtype((np.void, keys.dtype.itemsize * 3))).ravel()
    _, indices = np.unique(packed, return_index=True)
    return np.ascontiguousarray(points[np.sort(indices)], dtype=np.float32)


def read_transforms(conn, topic_ids, deserialize_message, TFMessage):
    relevant = {EDGE_ODOM, EDGE_LIDAR1, EDGE_LIDAR2}
    records = {edge: [] for edge in relevant}
    for topic in ("/tf_static", "/tf"):
        if topic not in topic_ids:
            continue
        sql = "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp"
        for (blob,) in conn.execute(sql, (topic_ids[topic],)):
            msg = deserialize_message(blob, TFMessage)
            for tf in msg.transforms:
                edge = (tf.header.frame_id.strip("/"), tf.child_frame_id.strip("/"))
                if edge in relevant:
                    t, q = tf.transform.translation, tf.transform.rotation
                    records[edge].append((stamp_s(tf.header.stamp),
                        (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))
    for edge, values in records.items():
        if not values:
            raise RuntimeError(f"Required TF not found in bag: {edge[0]} -> {edge[1]}")
        values.sort(key=lambda x: x[0])
    return records


def interpolate_tf(records, at_s, max_gap_s):
    ts, translations, quats = records
    i = int(np.searchsorted(ts, at_s, side="left"))
    if i == 0:
        if abs(float(ts[0]) - at_s) > max_gap_s:
            return None
        return to_matrix(translations[0], quats[0])
    if i >= len(ts):
        if abs(at_s - float(ts[-1])) > max_gap_s:
            return None
        return to_matrix(translations[-1], quats[-1])
    before, after = i - 1, i
    dt0 = at_s - float(ts[before])
    dt1 = float(ts[after]) - at_s
    if max(dt0, dt1) > max_gap_s:
        return None
    span = float(ts[after] - ts[before])
    ratio = 0.0 if span <= 1e-10 else dt0 / span
    tr = (1 - ratio) * translations[before] + ratio * translations[after]
    q = slerp(quats[before], quats[after], ratio)
    return to_matrix(tr, q)


def write_binary_ply(out: Path, points: np.ndarray):
    with out.open("wb") as f:
        f.write(("ply\nformat binary_little_endian 1.0\n"
                 "comment ECHO S02 LiDAR map in odom frame\n"
                 f"element vertex {len(points)}\n"
                 "property float x\nproperty float y\nproperty float z\n"
                 "end_header\n").encode("ascii"))
        f.write(np.asarray(points, dtype="<f4", order="C").tobytes())


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--bag", type=Path, default=DEFAULT_BAG)
    p.add_argument("--output", type=Path, help="New PLY path (must not exist)")
    p.add_argument("--stride", type=int, default=5, help="Use every Nth scan, default 5")
    p.add_argument("--max-range", type=float, default=10., help="LiDAR Euclidean range in metres")
    p.add_argument("--min-range", type=float, default=0.5)
    p.add_argument("--voxel", type=float, default=0.05, help="Voxel side length in metres")
    p.add_argument("--max-tf-gap", type=float, default=0.25, help="Maximum TF lookup gap in seconds")
    a = p.parse_args()
    if a.stride <= 0 or a.voxel <= 0 or a.max_range <= a.min_range:
        p.error("stride and voxel must be positive, and max-range > min-range")
    if not a.bag.is_dir():
        p.error(f"ROS bag directory missing: {a.bag}")
    bag_files = sorted(a.bag.glob("*.db3"))
    if len(bag_files) != 1:
        p.error(f"Expected exactly one SQLite bag .db3, found {len(bag_files)}")
    output = a.output or a.bag.parent / f"lidar_map_odom_{a.max_range:g}m_{a.voxel:g}m_stride{a.stride}.ply"
    output = output.expanduser().resolve()
    report = output.with_suffix(".json")
    if output.exists() or report.exists():
        p.error(f"Refusing to overwrite existing output: {output} or {report}")
    if not output.parent.is_dir():
        p.error(f"Output directory does not exist: {output.parent}")

    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from tf2_msgs.msg import TFMessage
    from sensor_msgs_py import point_cloud2

    print("=== ECHO S02 OFFLINE LIDAR RECONSTRUCTION ===", flush=True)
    print(f"Bag: {bag_files[0]}")
    print(f"Frame: odom | stride={a.stride} | range={a.min_range}–{a.max_range} m | voxel={a.voxel} m", flush=True)

    con = sqlite3.connect(f"file:{bag_files[0]}?mode=ro", uri=True)
    try:
        con.execute("PRAGMA query_only=ON")
        topic_ids = dict(con.execute("SELECT name, id FROM topics"))
        required = "/front_3d_lidar/lidar_points"
        if required not in topic_ids:
            raise RuntimeError(f"Bag does not contain {required}")
        print("[ECHO] Reading recorded TF...", flush=True)
        raw = read_transforms(con, topic_ids, deserialize_message, TFMessage)
        base_lidar = np.eye(4)
        for edge in (EDGE_LIDAR1, EDGE_LIDAR2):
            values = raw[edge]
            first = values[0]
            max_xyz_delta = max(np.linalg.norm(np.asarray(row[1]) - np.asarray(first[1])) for row in values)
            max_q_delta = max(1. - abs(float(np.dot(np.asarray(row[2]), np.asarray(first[2])))) for row in values)
            if max_xyz_delta > 0.001 or max_q_delta > 0.0001:
                raise RuntimeError(f"Sensor mounting TF changed over time: {edge}")
            base_lidar = base_lidar @ to_matrix(first[1], first[2])
            print(f"[ECHO] {edge[0]} -> {edge[1]}: t={first[1]} q={first[2]}", flush=True)
        trajectory = raw[EDGE_ODOM]
        timestamps = np.asarray([x[0] for x in trajectory], dtype=np.float64)
        translations = np.asarray([x[1] for x in trajectory], dtype=np.float64)
        quaternions = np.asarray([x[2] for x in trajectory], dtype=np.float64)
        print(f"[ECHO] Loaded {len(trajectory)} odom->base_link transforms", flush=True)

        selected = 0
        inspected = 0
        skipped_tf = 0
        bad_frame = 0
        accumulated = []
        raw_points_kept = 0
        first_scan = last_scan = None
        sql = "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp"
        for (blob,) in con.execute(sql, (topic_ids[required],)):
            inspected += 1
            if (inspected - 1) % a.stride != 0:
                continue
            scan = deserialize_message(blob, PointCloud2)
            if scan.header.frame_id.strip("/") != "front_3d_lidar":
                bad_frame += 1
                continue
            when = stamp_s(scan.header.stamp)
            base = interpolate_tf((timestamps, translations, quaternions), when, a.max_tf_gap)
            if base is None:
                skipped_tf += 1
                continue
            xyz = np.asarray(point_cloud2.read_points_numpy(
                scan, field_names=("x", "y", "z"), skip_nans=True)).reshape(-1, 3)
            norm2 = np.einsum("ij,ij->i", xyz, xyz)
            mask = np.isfinite(norm2) & (norm2 >= a.min_range*a.min_range) & (norm2 <= a.max_range*a.max_range)
            xyz = xyz[mask]
            if not len(xyz):
                continue
            transform = base @ base_lidar
            mapped = xyz @ transform[:3, :3].T + transform[:3, 3]
            compact = voxel_filter(mapped, a.voxel)
            accumulated.append(compact)
            raw_points_kept += len(xyz)
            selected += 1
            first_scan = when if first_scan is None else first_scan
            last_scan = when
            if selected % 20 == 0:
                print(f"[ECHO] {selected} scans transformed ({raw_points_kept:,} raw points within range)", flush=True)
            # Incremental compaction limits temporary point-cloud memory use.
            if sum(len(arr) for arr in accumulated) > 1_000_000:
                accumulated = [voxel_filter(np.concatenate(accumulated, axis=0), a.voxel)]
        if bad_frame:
            raise RuntimeError(f"Unexpected LiDAR frame on {bad_frame} selected scans")
        if selected < 3:
            raise RuntimeError(f"Only {selected} scans usable; TF time stamps may not align (skipped TF: {skipped_tf})")
        points = voxel_filter(np.concatenate(accumulated, axis=0), a.voxel)
        if not len(points):
            raise RuntimeError("Output point cloud empty")
        report_data = {
            "source_bag": str(a.bag.resolve()), "source_db3": str(bag_files[0]),
            "output_frame": "odom", "output_ply": str(output),
            "lidar_frame": "front_3d_lidar", "selected_scan_count": selected,
            "total_scan_count": inspected, "skipped_tf_count": skipped_tf,
            "selected_first_stamp_s": first_scan, "selected_last_stamp_s": last_scan,
            "raw_points_after_range_filter": raw_points_kept,
            "voxelized_points": len(points), "voxel_m": a.voxel,
            "max_range_m": a.max_range, "min_range_m": a.min_range,
            "stride": a.stride, "tf_max_gap_s": a.max_tf_gap,
            "xyz_min_m": points.min(axis=0).astype(float).tolist(),
            "xyz_max_m": points.max(axis=0).astype(float).tolist(),
            "coordinate_note": "LiDAR map uses odom frame; RTAB-Map RGB-D exports may use map frame and require alignment.",
        }
        write_binary_ply(output, points)
        report.write_text(json.dumps(report_data, indent=2) + "\n")
        print("\n=== ECHO S02 LIDAR MAP RESULT ===")
        print(f"Processed LiDAR scans: {selected} / {inspected} (TF skips: {skipped_tf})")
        print(f"Valid in-range points before voxelization: {raw_points_kept:,}")
        print(f"Output points (5 cm voxels by default): {len(points):,}")
        print(f"Map bounds min XYZ: {report_data['xyz_min_m']}")
        print(f"Map bounds max XYZ: {report_data['xyz_max_m']}")
        print(f"PLY: {output}")
        print(f"Metadata: {report}")
        print("[PASS] Read-only offline reconstruction completed")
    finally:
        con.close()


if __name__ == "__main__":
    main()
