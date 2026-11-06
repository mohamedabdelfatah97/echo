import json
from pathlib import Path

import numpy as np

ROOT = Path.home() / "raafif"
DATA = ROOT / "data"
MASKS = ROOT / "results/experiment_02"
OUT = ROOT / "results/experiment_03"
OUT.mkdir(parents=True, exist_ok=True)


def estimate_surface_center(label, name):
    depth = np.load(DATA / label / "depth.npy")

    mask = np.load(
        MASKS / f"{label}_{name}_mask.npy"
    ).astype(bool)

    camera = json.loads(
        (DATA / label / "camera.json").read_text()
    )

    if mask.shape != depth.shape:
        raise RuntimeError("Depth and mask shapes differ")

    valid = (
        mask &
        np.isfinite(depth) &
        (depth > 0)
    )

    v, u = np.nonzero(valid)
    z = depth[valid].astype(np.float64)

    if len(z) < 100:
        raise RuntimeError(
            f"Not enough valid points: {label}/{name}"
        )

    fx = float(camera["fx"])
    fy = float(camera["fy"])
    cx = float(camera["cx"])
    cy = float(camera["cy"])

    # USD camera convention:
    # X right, Y up, -Z forward.
    x_cam = (u - cx) * z / fx
    y_cam = -(v - cy) * z / fy
    z_cam = -z

    # Homogeneous points in camera coordinates
    points = np.column_stack([
        x_cam, y_cam, z_cam,
        np.ones_like(z)
    ])

    # USD Gf.Matrix4d uses row-vector convention.
    cam_to_world = np.array(
        camera["camera_to_world_usd"],
        dtype=np.float64
    )

    world = (points @ cam_to_world)[:, :3]

    # Robust proxy for center of visible object surface
    center = np.median(world, axis=0)

    scale = float(
        camera.get("stage_meters_per_unit", 1.0)
    )

    return center * scale, len(z)


results = {}

for name in ("left_box", "right_box"):
    before, count0 = estimate_surface_center(
        "before", name
    )

    after, count1 = estimate_surface_center(
        "after", name
    )

    displacement = after - before
    distance = np.linalg.norm(displacement)

    results[name] = {
        "before_m": before.tolist(),
        "after_m": after.tolist(),
        "displacement_xyz_m": displacement.tolist(),
        "distance_m": float(distance),
        "valid_pixels_before": count0,
        "valid_pixels_after": count1
    }

    print(f"\n=== {name.upper()} ===")
    print("Before XYZ (m):", np.round(before, 4))
    print("After XYZ  (m):", np.round(after, 4))
    print("Delta XYZ  (m):", np.round(displacement, 4))
    print("Movement   (m):", round(distance, 4))


# Simulator ground truth for the moved object
gt0 = json.loads(
    (DATA / "ground_truth/before.json").read_text()
)

gt1 = json.loads(
    (DATA / "ground_truth/after.json").read_text()
)

m0 = np.array(gt0["object_to_world_usd"])
m1 = np.array(gt1["object_to_world_usd"])

# USD translation occupies the last matrix row
ground_truth_delta = m1[3, :3] - m0[3, :3]

scale = json.loads(
    (DATA / "before/camera.json").read_text()
).get("stage_meters_per_unit", 1.0)

ground_truth_delta *= float(scale)

estimated_delta = np.array(
    results["right_box"]["displacement_xyz_m"]
)

vector_error = np.linalg.norm(
    estimated_delta - ground_truth_delta
)

results["ground_truth"] = {
    "displacement_xyz_m": ground_truth_delta.tolist(),
    "distance_m": float(
        np.linalg.norm(ground_truth_delta)
    ),
    "estimated_vector_error_m": float(vector_error)
}

print("\n=== RAAFIF 3D MOVEMENT EVALUATION ===")
print(
    "Ground truth (m):",
    np.round(ground_truth_delta, 4)
)
print(
    "Estimated (m):",
    np.round(estimated_delta, 4)
)
print(
    "Vector error (m):",
    round(vector_error, 4)
)

(OUT / "movement_results.json").write_text(
    json.dumps(results, indent=2)
)

print("\nSaved:", OUT / "movement_results.json")
