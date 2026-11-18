#!/usr/bin/env python3
"""ECHO M04D: offline ground-truth-gated LiDAR coverage audit."""

import csv
import math
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

ROOT = Path.home() / "echo/diagnostics/m04d"
BAG = ROOT / "m04d_gt_window_20261118T122549Z"
GT = ROOT / "ground_truth/m04d_20261118T122549Z.csv"
OUT = ROOT / "analysis"
OUT.mkdir(parents=True, exist_ok=True)

with GT.open(newline="") as f:
    gt = list(csv.DictReader(f))

times = np.array([float(r["sim_time_s"]) for r in gt])
positions = np.array([
    [float(r[k]) for k in ("x_m", "y_m", "z_m")]
    for r in gt
])

def stamp(s):
    return s.sec + s.nanosec * 1e-9

def rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x*x+y*y+z*z+w*w)
    x, y, z, w = x/n, y/n, z/n, w/n

    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
    ])

# Provisional calibration obtained in the same sim session,
# but after the recorded interval.
theta = math.radians(179.998)
Rwo = np.array([
    [math.cos(theta), -math.sin(theta), 0],
    [math.sin(theta), math.cos(theta), 0],
    [0, 0, 1]
])
two = np.array([-16.65545, -4.37954, 0.0])

edges = [
    ("odom", "base_link"),
    ("base_link", "XT_32"),
    ("XT_32", "front_3d_lidar"),
]

reader = rosbag2_py.SequentialReader()
reader.open(
    rosbag2_py.StorageOptions(
        uri=str(BAG), storage_id="mcap"
    ),
    rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )
)

latest_tf = {}
results = []
skipped = 0

print("=== M04D FULL LIDAR COVERAGE AUDIT ===")
print(f"GT interval: {times[0]:.3f} - {times[-1]:.3f} s")

while reader.has_next():
    topic, raw, bag_ns = reader.read_next()

    if bag_ns * 1e-9 > times[-1] + 0.5:
        break

    if topic == "/tf":
        msg = deserialize_message(raw, TFMessage)
        for tr in msg.transforms:
            edge = (tr.header.frame_id, tr.child_frame_id)
            if edge in edges:
                latest_tf[edge] = (stamp(tr.header.stamp), tr)
        continue

    if topic != "/front_3d_lidar/lidar_points":
        continue

    cloud = deserialize_message(raw, PointCloud2)
    t = stamp(cloud.header.stamp)

    if t < times[0] or t > times[-1]:
        continue

    index = int(np.argmin(np.abs(times - t)))
    dt = abs(times[index] - t)

    if dt > 0.06 or not all(e in latest_tf for e in edges):
        skipped += 1
        continue

    tf_dt = max(abs(latest_tf[e][0]-t) for e in edges)

    if tf_dt > 0.1:
        skipped += 1
        continue

    R = np.eye(3)
    p = np.zeros(3)

    for edge in edges:
        tr = latest_tf[edge][1].transform
        translation = np.array([
            tr.translation.x,
            tr.translation.y,
            tr.translation.z
        ])
        p += R @ translation
        R = R @ rot(tr.rotation)

    worker_odom = Rwo.T @ (positions[index] - two)
    expected = R.T @ (worker_odom - p)

    xyz = point_cloud2.read_points_numpy(
        cloud,
        field_names=("x", "y", "z"),
        skip_nans=True
    )

    dxy = np.linalg.norm(xyz[:, :2]-expected[:2], axis=1)
    height = xyz[:, 2]-expected[2]

    mask = (
        (dxy <= 0.5) &
        (height >= 0.2) &
        (height <= 2.2)
    )

    cluster = xyz[mask]
    count = len(cluster)

    center_error = float("nan")
    residual_x = float("nan")
    residual_y = float("nan")

    if count:
        center = np.median(cluster[:, :2], axis=0)
        residual = center - expected[:2]

        residual_x, residual_y = map(float, residual)
        center_error = float(np.linalg.norm(residual))

    results.append({
        "sim_time_s": t,
        "gt_time_error_ms": dt * 1000,
        "tf_time_error_ms": tf_dt * 1000,
        "worker_x_world": positions[index, 0],
        "worker_y_world": positions[index, 1],
        "expected_x_lidar": expected[0],
        "expected_y_lidar": expected[1],
        "nearby_points": count,
        "center_error_m": center_error,
        "residual_x_m": residual_x,
        "residual_y_m": residual_y,
    })

assert results, "No valid LiDAR frames found"

path = OUT / "m04d_worker_lidar_coverage.csv"

with path.open("w", newline="") as f:
    writer = csv.DictWriter(
        f, fieldnames=list(results[0].keys())
    )
    writer.writeheader()
    writer.writerows(results)

counts = np.array([r["nearby_points"] for r in results])
errors = np.array([
    r["center_error_m"] for r in results
    if np.isfinite(r["center_error_m"])
])

print("\n=== COVERAGE RESULTS ===")
print("Frames evaluated:", len(results))
print("Frames skipped:", skipped)
print("Frames with >= 30 nearby points:",
      int(np.count_nonzero(counts >= 30)))
print("Coverage fraction:",
      f"{np.mean(counts >= 30)*100:.1f}%")
print("Nearby points median:", float(np.median(counts)))
print("Nearby points minimum:", int(counts.min()))

if len(errors):
    print("Median center error:",
          round(float(np.median(errors)), 3), "m")
    print("95th percentile center error:",
          round(float(np.percentile(errors, 95)), 3), "m")

print("\nResults CSV:", path)
print("AUDIT COMPLETE")
