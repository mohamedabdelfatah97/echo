#!/usr/bin/env python3
"""M04D: LiDAR-only motion-candidate preview. No GT input."""

from pathlib import Path
import math

import numpy as np
import rosbag2_py

from scipy.spatial import cKDTree
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

ROOT = Path.home() / "echo"
BAG = ROOT / "diagnostics/m04d/m04d_gt_window_20261118T122549Z"
OUT = ROOT / "diagnostics/m04d/analysis"

OUT.mkdir(parents=True, exist_ok=True)

TARGETS = [1309.3, 1314.3]

EDGES = [
    ("odom", "base_link"),
    ("base_link", "XT_32"),
    ("XT_32", "front_3d_lidar"),
]


def stamp(s):
    return s.sec + s.nanosec * 1e-9


def rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n

    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(y*z+x*w), 2*(x*x-y*w), 2*(y*z-x*w)]
    ]) if False else np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
    ])


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
samples = {}

print("=== M04D LIDAR-ONLY MOTION PREVIEW ===")

while reader.has_next():
    topic, raw, bag_ns = reader.read_next()

    if bag_ns * 1e-9 > max(TARGETS) + 0.5:
        break

    if topic == "/tf":
        msg = deserialize_message(raw, TFMessage)

        for tr in msg.transforms:
            edge = (tr.header.frame_id, tr.child_frame_id)

            if edge in EDGES:
                latest_tf[edge] = (
                    stamp(tr.header.stamp), tr
                )
        continue

    if topic != "/front_3d_lidar/lidar_points":
        continue

    msg = deserialize_message(raw, PointCloud2)
    t = stamp(msg.header.stamp)

    for index, target in enumerate(TARGETS):
        if index in samples or abs(t-target) > 0.04:
            continue

        if not all(e in latest_tf for e in EDGES):
            continue

        tf_age = max(
            abs(latest_tf[e][0]-t) for e in EDGES
        )

        if tf_age > 0.1:
            continue

        # Compose LiDAR pose in odom frame.
        R = np.eye(3)
        p = np.zeros(3)

        for edge in EDGES:
            tr = latest_tf[edge][1].transform

            v = np.array([
                tr.translation.x,
                tr.translation.y,
                tr.translation.z
            ])

            p = p + R @ v
            R = R @ rotation(tr.rotation)

        xyz = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        # Transform points from LiDAR into odom.
        xyz_odom = xyz @ R.T + p

        # Limit analysis to useful above-floor returns.
        horizontal_range = np.linalg.norm(
            xyz_odom[:, :2] - p[:2], axis=1
        )

        mask = (
            np.all(np.isfinite(xyz_odom), axis=1)
            & (xyz_odom[:, 2] >= 0.2)
            & (xyz_odom[:, 2] <= 2.3)
            & (horizontal_range >= 1.0)
            & (horizontal_range <= 10.0)
        )

        samples[index] = (t, xyz_odom[mask])

        print(
            f"Scan {index+1}: t={t:.3f}, "
            f"filtered_points={int(mask.sum())}, "
            f"TF_age={tf_age*1000:.1f}ms"
        )

    if len(samples) == 2:
        break

assert len(samples) == 2, "Could not locate both scans"

t0, reference = samples[0]
t1, current = samples[1]

# Compare current points with the previous scan
# in the same odom coordinate frame.
tree = cKDTree(reference)

distances, _ = tree.query(
    current, k=1, workers=-1
)

threshold = 0.25
candidates = current[distances > threshold]

print("\n=== TEMPORAL CHANGE RESULTS ===")
print(f"Time separation: {t1-t0:.3f} s")
print("Reference points:", len(reference))
print("Current points:", len(current))
print("Changed points:", len(candidates))
print("Change threshold:", threshold, "m")

# Spatial histogram of candidate regions.
if len(candidates):
    grid = np.floor(
        candidates[:, :2] / 0.5
    ).astype(int)

    cells, counts = np.unique(
        grid, axis=0, return_counts=True
    )

    order = np.argsort(counts)[::-1]

    print("\nMost populated 0.5 m change cells:")

    for i in order[:10]:
        center = (cells[i] + 0.5) * 0.5

        print(
            f"  XY=({center[0]:+.2f},"
            f" {center[1]:+.2f}), "
            f"points={counts[i]}"
        )

# Save a visual preview.
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rng = np.random.default_rng(123)

indices = rng.choice(
    len(current),
    size=min(25000, len(current)),
    replace=False
)

fig, ax = plt.subplots(figsize=(10, 8))

ax.scatter(
    current[indices, 0],
    current[indices, 1],
    s=0.5,
    c="lightgray",
    label="Current LiDAR cloud"
)

ax.scatter(
    candidates[:, 0],
    candidates[:, 1],
    s=2,
    c="red",
    label="Change candidates"
)

ax.set_aspect("equal")
ax.set_xlabel("Odom X (m)")
ax.set_ylabel("Odom Y (m)")
ax.set_title(
    "ECHO M04D — LiDAR temporal change candidates"
)
ax.grid(alpha=0.2)
ax.legend()

png = OUT / "m04d_lidar_motion_preview.png"

fig.savefig(png, dpi=180, bbox_inches="tight")
plt.close(fig)

print("\nPreview:", png)
print("MOTION PREVIEW COMPLETE")
