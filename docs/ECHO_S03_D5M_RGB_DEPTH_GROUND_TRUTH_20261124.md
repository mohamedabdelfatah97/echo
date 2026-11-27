# ECHO S03-D5M — RGB and Native Depth Sphere Ground-Truth Review

**Review date:** 2026-11-24  
**Input:** `echo_s03_d5m_gt_20261123T233116Z.zip`  
**Method:** Independent offline replay of target-mask predictions against archived original RGB, native depth, known sphere poses and the D5M source. No ECHO repository or camera settings modified by this review.

## Scope and observations

- Static front RGB and pseudo-depth cameras in the four-D455 Isaac Sim scene, headless in-process Replicator capture, **not ROS transport or RTAB-Map**.
- Nine magenta spheres, radius 0.045 m, centered approximately 1.75 m forward in the front RGB optical frame, observed in both cameras. Native RGB and depth centroids are individually reported for all nine targets.
- 640 × 480 images; fixed `fx = fy = 317.04311998 px`; observed RGB sphere-mask pixels 2,031; observed depth sphere-mask pixels 1,908. The depth-to-RGB camera origin separation was measured as 0.0115 m.
- Camera translation and orientation did not change between the scripted measurements and captures at the reported precision. This does not measure ROS exposure/timestamp synchronization.

## Candidate comparison (replayed independently)

| Candidate | Depth `(cx,cy)` | RGB `(cx,cy)` | RGB silhouette IoU | Native depth silhouette IoU | Native depth surface RMSE (mm) |
|---|---|---|---:|---:|---:|
| Baseline | (320,240) | (320,240) | 0.899713 | 0.893648 | 2.92740 |
| Coupled Y | (320,239.5) | (320,239.5) | 0.929714 | 0.933801 | 2.54758 |
| **Coupled XY** | **(319.5,239.5)** | **(319.5,239.5)** | **0.968012** | **0.972477** | **2.08573** |
| Depth-only XY | (319.5,239.5) | (320,240) | 0.899713 | 0.972477 | 2.08573 |

The coupled XY model improves the RGB silhouette score by **6.83 percentage points** and depth silhouette score by **7.88 percentage points** over baseline. The reported native depth surface RMSE decreases **0.842 mm (28.8%)**.

### Additional common-sample check

The source implementation evaluates each candidate's surface residuals on that candidate's own eroded, predicted-and-observed intersection mask. Those sample sizes differ (baseline 1,275; coupled XY 1,344 pixels). For a fairer comparison, an independent offline check used **the same 1,422 target-interior pixels**: the intersection of observed interior and both candidate predicted interiors. Results:

| Candidate | Same interior samples | RMSE (mm) | Mean signed residual (mm) |
|---|---:|---:|---:|
| Baseline | 1,422 | **3.151** | +1.939 |
| Coupled XY | 1,422 | **2.131** | +1.957 |

On the shared pixels, coupled XY still improves RMSE by **1.020 mm (32.4%)**. Its residual remains systematically positive, so **2.1 mm is a simulator/rendering surface-model residual, not a proven physical D455 measurement error**. Sphere tessellation, rasterization and the analytical sphere model may contribute; this run does not isolate these causes.

Each of the nine individual target-mask IoUs improves for **both RGB and depth** when switching from baseline to coupled XY. This makes the fit improvement more credible than a single target near the image center.

## What the perfect cross-view score does and does not mean

The source reports `projected_exact_rgb_magenta_fraction = 1.0` for **baseline, coupled Y and coupled XY**, and `0.967505` for depth-only XY. The metric asks whether projected native depth target pixels land **inside any RGB magenta region**, not how close they land to the true silhouette boundary or each sphere center. Because the spheres have finite area and adjacent candidate shifts are small, this metric **cannot distinguish baseline from coupled XY**. The independently predicted RGB and native depth sphere silhouettes, not the perfect cross-view inclusion value, supply the discriminating evidence.

## Interpretation and limits

The evidence now strongly favors **`cx=319.5`, `cy=239.5` as effective pixel-index coordinates for this front RGB and native depth rendering configuration**, using the published focal length. It is compatible with a conventional half-pixel distinction: pixel-index coordinates 0..639 / 0..479 have midpoint (319.5,239.5), while pixel-corner or half-integer-center coordinates can describe the same optical center as (320,240). The experiment does not alone demonstrate that every consumer of `sensor_msgs/CameraInfo` expects the shifted values. The ROS downstream consumer convention is a separate validation requirement.

This run uses only the front camera, one target distance, one stationary pose and in-process Replicator captures. It does not prove four-camera agreement, native ROS camera-info/TF synchronization, the quality of registered RGB-D streams delivered to RTAB-Map, or correct sensor accuracy on real D455 hardware. There is no evidence that the old raw sensor samples were changed by these candidate calculations; these are **offline interpretations** of identical archived data.

## Decision and next gate

**S03-D5M: PASSED as a preliminary front, in-process ground-truth geometry comparison.** The coupled XY candidate is justified for **isolated opt-in ROS validation**; do not silently replace the verified baseline yet.

Recommended next gate: inspect current repo/ROS camera relay before edits; capture a native, synchronized RGB+depth+CameraInfo+TF baseline; introduce a **temporary separate-namespace** coupled-XY CameraInfo candidate with unchanged image samples, then compare actual back-projected geometry and, subsequently, the four-camera RTAB-Map pipeline against the untouched baseline. Validate all four camera extrinsics/timing and at least one known static feature or plane, and record quantitative differences. Retain the verified 3D LiDAR and navigation baselines.

## Data integrity

D5M script reports `stage_saved=false` and no Git/ROS/CameraInfo changes; this is supported by inspection of the script (USD mutations made in session layer, no stage save call), but the remote repository working-tree status was **not** independently checked. Archive the original D5M ZIP and this review outside the repository or commit only after a separate source-control check.
