import asyncio
import json
from pathlib import Path

import numpy as np
from PIL import Image
import omni.usd
import omni.timeline
import omni.kit.app

from pxr import UsdGeom
from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor

# Select before or after from the Script Editor
LABEL = globals().get("CAPTURE_LABEL", "before")

if LABEL not in ("before", "after"):
    raise ValueError("LABEL must be before or after")

ROOT = Path.home() / "raafif" / "data"
OUT = ROOT / LABEL
GT = ROOT / "ground_truth"

OUT.mkdir(parents=True, exist_ok=True)
GT.mkdir(parents=True, exist_ok=True)

CAM_PATH = "/World/RAAFIF_Camera"
BOX_PATH = "/World/RAAFIF_Office/SM_Box_45"

stage = omni.usd.get_context().get_stage()

for path in (CAM_PATH, BOX_PATH):
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError(f"Missing prim: {path}")

rtx = RtxCamera(
    path=CAM_PATH,
    reset_xform_op_properties=False
)

sensor = CameraSensor(
    rtx,
    resolution=(480, 640),
    annotators=["rgb", "distance_to_image_plane"]
)

async def capture():
    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()

    try:
        if not was_playing:
            timeline.play()

        rgb = depth = None

        for _ in range(180):
            await omni.kit.app.get_app().next_update_async()

            r, _ = sensor.get_data("rgb")
            d, _ = sensor.get_data("distance_to_image_plane")

            if r is None or d is None:
                continue

            rgb = np.squeeze(np.asarray(r.numpy())).copy()
            depth = np.squeeze(
                np.asarray(d.numpy())
            ).astype(np.float32).copy()

            if rgb.size and depth.size:
                break

        if rgb is None or depth is None:
            raise RuntimeError("No RGB-D frame received")

        if rgb.ndim != 3 or depth.ndim != 2:
            raise RuntimeError("Unexpected camera shapes")

        rgb = rgb[:, :, :3].astype(np.uint8)

        valid = np.isfinite(depth) & (depth > 0)
        if valid.sum() == 0:
            raise RuntimeError("No valid depth")

        Image.fromarray(rgb).save(OUT / "rgb.png")
        np.save(OUT / "depth.npy", depth)

        # Auto-contrast depth for better visualization
        low, high = np.percentile(depth[valid], [5, 95])
        high = max(high, low + 0.001)

        preview = np.zeros(depth.shape, dtype=np.uint8)
        normalized = np.clip(
            (depth[valid] - low) / (high - low), 0, 1
        )
        preview[valid] = (255 * (1 - normalized)).astype(
            np.uint8
        )
        Image.fromarray(preview).save(
            OUT / "depth_preview.png"
        )

        cache = UsdGeom.XformCache()

        def world_matrix(path):
            matrix = cache.GetLocalToWorldTransform(
                stage.GetPrimAtPath(path)
            )
            return [
                [float(matrix[i][j]) for j in range(4)]
                for i in range(4)
            ]

        cam = UsdGeom.Camera(
            stage.GetPrimAtPath(CAM_PATH)
        )

        h, w = depth.shape
        focal = cam.GetFocalLengthAttr().Get()
        ha = cam.GetHorizontalApertureAttr().Get()
        va = cam.GetVerticalApertureAttr().Get()

        metadata = {
            "resolution": [h, w],
            "camera_path": CAM_PATH,
            "fx": focal / ha * w,
            "fy": focal / va * h,
            "cx": w / 2,
            "cy": h / 2,
            "depth_type": "distance_to_image_plane",
            "stage_meters_per_unit": float(
                UsdGeom.GetStageMetersPerUnit(stage)
            ),
            "camera_to_world_usd": world_matrix(CAM_PATH)
        }

        (OUT / "camera.json").write_text(
            json.dumps(metadata, indent=2)
        )

        ground_truth = {
            "object_path": BOX_PATH,
            "object_to_world_usd": world_matrix(BOX_PATH)
        }

        (GT / f"{LABEL}.json").write_text(
            json.dumps(ground_truth, indent=2)
        )

        print(f"\n=== RAAFIF {LABEL.upper()} SUCCESS ===")
        print("RGB:", rgb.shape)
        print("Depth:", depth.shape)
        print("Median depth:", float(np.median(depth[valid])))
        print("Saved:", OUT)

    except Exception:
        import traceback
        traceback.print_exc()

    finally:
        if not was_playing:
            timeline.stop()

asyncio.ensure_future(capture())
