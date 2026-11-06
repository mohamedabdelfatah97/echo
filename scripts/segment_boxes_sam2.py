from pathlib import Path
import json
import numpy as np
import torch
from PIL import Image, ImageDraw

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

ROOT = Path.home() / "raafif"
SAM = ROOT / "third_party" / "sam2"
OUT = ROOT / "results" / "experiment_02"
OUT.mkdir(parents=True, exist_ok=True)

CHECKPOINT = str(SAM / "checkpoints/sam2.1_hiera_small.pt")
CONFIG = "configs/sam2.1/sam2.1_hiera_s.yaml"

# Prompts based on our actual RAAFIF images.
# Boxes: [x_min, y_min, x_max, y_max]
# Positive point: inside target object
# Negative point: inside neighboring object

PROMPTS = {
    "before": {
        "left_box": {
            "box": [270, 103, 417, 298],
            "positive": [335, 190],
            "negative": [485, 190]
        },
        "right_box": {
            "box": [416, 105, 556, 295],
            "positive": [485, 190],
            "negative": [335, 190]
        }
    },
    "after": {
        "left_box": {
            "box": [269, 106, 415, 298],
            "positive": [315, 190],
            "negative": [455, 190]
        },
        "right_box": {
            "box": [363, 105, 489, 292],
            "positive": [455, 190],
            "negative": [315, 190]
        }
    }
}

print("=== RAAFIF EXPERIMENT 2 ===")
print("GPU:", torch.cuda.get_device_name(0))
print("Loading SAM 2.1 Small...")

model = build_sam2(CONFIG, CHECKPOINT, device="cuda")
predictor = SAM2ImagePredictor(model)

results = {}

COLORS = {
    "left_box": np.array([40, 220, 80]),
    "right_box": np.array([245, 65, 65])
}

with torch.inference_mode(), torch.autocast(
    "cuda", dtype=torch.bfloat16
):
    for label in ("before", "after"):
        image_path = ROOT / "data" / label / "rgb.png"
        image = np.asarray(
            Image.open(image_path).convert("RGB")
        )

        if image.shape != (480, 640, 3):
            raise RuntimeError(
                f"Unexpected image shape: {image.shape}"
            )

        print(f"\nProcessing {label} image...")
        predictor.set_image(image)

        overlay = image.copy()
        results[label] = {}

        for name, prompt in PROMPTS[label].items():
            points = np.array([
                prompt["positive"],
                prompt["negative"]
            ], dtype=np.float32)

            point_labels = np.array(
                [1, 0], dtype=np.int32
            )

            masks, scores, _ = predictor.predict(
                point_coords=points,
                point_labels=point_labels,
                box=np.array(
                    prompt["box"], dtype=np.float32
                ),
                multimask_output=False
            )

            mask = masks[0].astype(bool)
            score = float(scores[0])

            mask_path = OUT / f"{label}_{name}_mask.npy"
            np.save(mask_path, mask)

            Image.fromarray(
                (mask * 255).astype(np.uint8)
            ).save(OUT / f"{label}_{name}_mask.png")

            color = COLORS[name]
            overlay[mask] = (
                0.5 * overlay[mask].astype(np.float32)
                + 0.5 * color
            ).astype(np.uint8)

            ys, xs = np.nonzero(mask)
            centroid = (
                [float(xs.mean()), float(ys.mean())]
                if len(xs) else None
            )

            results[label][name] = {
                "prompt": prompt,
                "score": score,
                "mask_pixels": int(mask.sum()),
                "centroid_px": centroid
            }

            print(
                f"{name}: score={score:.3f}, "
                f"pixels={mask.sum()}, "
                f"centroid={centroid}"
            )

        Image.fromarray(overlay).save(
            OUT / f"{label}_segmented.png"
        )

        # Show where the prompts were placed
        preview = Image.fromarray(image.copy())
        draw = ImageDraw.Draw(preview)

        for name, prompt in PROMPTS[label].items():
            color = tuple(int(c) for c in COLORS[name])
            draw.rectangle(
                prompt["box"], outline=color, width=2
            )
            x, y = prompt["positive"]
            draw.ellipse(
                [x-4, y-4, x+4, y+4],
                fill=color
            )

        preview.save(OUT / f"{label}_prompts.png")

(OUT / "segmentation_results.json").write_text(
    json.dumps(results, indent=2)
)

before = Image.open(OUT / "before_segmented.png")
after = Image.open(OUT / "after_segmented.png")

comparison = Image.new(
    "RGB", (before.width + after.width, before.height)
)
comparison.paste(before, (0, 0))
comparison.paste(after, (before.width, 0))
comparison.save(OUT / "comparison.png")

print("\n=== EXPERIMENT 2 COMPLETE ===")
print("Results:", OUT)
print("Inspect comparison.png")
