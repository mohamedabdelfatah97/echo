from pathlib import Path
import json

import cv2
import numpy as np
import torch
from PIL import Image

from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

ROOT = Path.home() / "raafif"
SAM = ROOT / "third_party/sam2"
OUT = ROOT / "results/experiment_05"
OUT.mkdir(parents=True, exist_ok=True)

model = build_sam2(
    "configs/sam2.1/sam2.1_hiera_s.yaml",
    str(SAM / "checkpoints/sam2.1_hiera_small.pt"),
    device="cuda"
)

generator = SAM2AutomaticMaskGenerator(
    model=model,
    points_per_side=24,
    points_per_batch=16,
    pred_iou_thresh=0.80,
    stability_score_thresh=0.90,
    crop_n_layers=0,
    min_mask_region_area=0,
    output_mode="binary_mask"
)

print("=== RAAFIF EXPERIMENT 5 ===")
print("Automatic SAM 2 segmentation")
print("GPU:", torch.cuda.get_device_name(0))

for label in ("before", "after"):
    rgb = np.array(
        Image.open(ROOT / "data" / label / "rgb.png").convert("RGB"),
        copy=True
    )

    print(f"\nGenerating proposals: {label}")

    with torch.inference_mode(), torch.autocast(
        "cuda", dtype=torch.bfloat16
    ):
        annotations = generator.generate(rgb)

    annotations.sort(
        key=lambda a: a["area"], reverse=True
    )

    masks = np.stack(
        [a["segmentation"].astype(bool) for a in annotations]
    ) if annotations else np.zeros(
        (0, *rgb.shape[:2]), dtype=bool
    )

    np.savez_compressed(
        OUT / f"{label}_all_masks.npz",
        masks=masks
    )

    overlay = rgb.copy()
    records = []

    for i, annotation in enumerate(annotations):
        x, y, w, h = annotation["bbox"]
        area = int(annotation["area"])
        cx, cy = x + w / 2, y + h / 2

        # Diagnostic selection near the experiment objects.
        # This does NOT affect automatic mask generation.
        selected = (
            2500 <= area <= 40000
            and 220 <= cx <= 590
            and 90 <= cy <= 325
        )

        records.append({
            "proposal_id": i,
            "area_pixels": area,
            "bbox_xywh": [float(x), float(y),
                          float(w), float(h)],
            "predicted_iou": float(
                annotation["predicted_iou"]
            ),
            "selected_for_display": selected
        })

        if not selected:
            continue

        mask = masks[i]
        color = np.array([
            (67 * i + 50) % 200 + 40,
            (97 * i + 90) % 200 + 40,
            (43 * i + 120) % 200 + 40
        ], dtype=np.uint8)

        overlay[mask] = (
            0.55 * overlay[mask].astype(float)
            + 0.45 * color
        ).astype(np.uint8)

        contours, _ = cv2.findContours(
            mask.astype(np.uint8),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        cv2.drawContours(
            overlay, contours, -1,
            tuple(int(c) for c in color), 2
        )

        cv2.putText(
            overlay, str(i),
            (int(cx), int(cy)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (0, 0, 0), 3
        )
        cv2.putText(
            overlay, str(i),
            (int(cx), int(cy)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 1
        )

    Image.fromarray(overlay).save(
        OUT / f"{label}_automatic.png"
    )

    (OUT / f"{label}_proposals.json").write_text(
        json.dumps(records, indent=2)
    )

    print("Total proposals:", len(annotations))
    print(
        "Displayed candidates:",
        sum(r["selected_for_display"] for r in records)
    )

before = Image.open(OUT / "before_automatic.png")
after = Image.open(OUT / "after_automatic.png")

comparison = Image.new(
    "RGB", (before.width + after.width, before.height)
)
comparison.paste(before, (0, 0))
comparison.paste(after, (before.width, 0))
comparison.save(OUT / "automatic_comparison.png")

print("\n=== EXPERIMENT 5 COMPLETE ===")
print("Saved:", OUT)
