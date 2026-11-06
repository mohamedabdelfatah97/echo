from pathlib import Path
import json
import numpy as np
from PIL import Image

ROOT = Path.home() / "raafif"
BEFORE = ROOT / "data/before"
AFTER = ROOT / "data/after"
OUT = ROOT / "results/experiment_01"
OUT.mkdir(parents=True, exist_ok=True)

# Load actual RGB-D observations
rgb0 = np.array(Image.open(BEFORE / "rgb.png").convert("RGB"))
rgb1 = np.array(Image.open(AFTER / "rgb.png").convert("RGB"))
d0 = np.load(BEFORE / "depth.npy")
d1 = np.load(AFTER / "depth.npy")

assert rgb0.shape == rgb1.shape
assert d0.shape == d1.shape == rgb0.shape[:2]

# Verify camera poses match
c0 = json.loads((BEFORE / "camera.json").read_text())
c1 = json.loads((AFTER / "camera.json").read_text())

p0 = np.array(c0["camera_to_world_usd"])
p1 = np.array(c1["camera_to_world_usd"])

if not np.allclose(p0, p1, atol=1e-4):
    raise RuntimeError(
        "Camera poses differ! Images must be registered first."
    )

# RGB difference
rgb_diff = np.max(
    np.abs(rgb0.astype(np.int16) - rgb1.astype(np.int16)),
    axis=2
)
rgb_changed = rgb_diff > 30

# Depth difference in stage units
valid0 = np.isfinite(d0) & (d0 > 0)
valid1 = np.isfinite(d1) & (d1 > 0)

depth_changed = valid0 ^ valid1
both_valid = valid0 & valid1

depth_changed[both_valid] = (
    np.abs(d0[both_valid] - d1[both_valid]) > 0.04
)

# Combine RGB and depth evidence
changed = rgb_changed | depth_changed

# Color-coded difference map:
# Red = RGB only, cyan = depth only, yellow = both
visual = np.zeros_like(rgb0)
visual[rgb_changed] = [255, 0, 0]
visual[depth_changed] = [0, 255, 255]
visual[rgb_changed & depth_changed] = [255, 255, 0]

# Overlay the detected changes on the after image
overlay = rgb1.copy()
overlay[changed] = (
    0.55 * rgb1[changed].astype(float)
    + 0.45 * np.array([255, 0, 0])
).astype(np.uint8)

Image.fromarray(visual).save(OUT / "change_map.png")
Image.fromarray(overlay).save(OUT / "change_overlay.png")

total = changed.size
num_changed = int(changed.sum())

print("\n=== RAAFIF CHANGE DETECTION ===")
print("Resolution:", rgb0.shape[1], "x", rgb0.shape[0])
print("RGB changed pixels:", int(rgb_changed.sum()))
print("Depth changed pixels:", int(depth_changed.sum()))
print("Combined changed pixels:", num_changed)
print("Changed area: %.2f%%" % (100 * num_changed / total))
print("Results:", OUT)
