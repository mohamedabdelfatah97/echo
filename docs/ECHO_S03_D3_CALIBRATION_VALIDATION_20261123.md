# ECHO — S03-D3 Four-D455 Calibration and Geometry Validation

**Date:** 23 November 2026  
**Status:** Calibration/TF consistency and preliminary cross-camera planar geometry validated; absolute simulator-ground-truth accuracy not yet established  
**Experiment:** White ROS-enabled Nova Carter with front, rear, left and right *simulated D455-style* RGB-D cameras, NVIDIA Isaac Sim 6.2, ROS 2 Jazzy, RTAB-Map  
**Repository:** `~/echo`  
**Branch / HEAD (confirmed by user terminal):** `echo/s01-d455` / `e7f83b4` (`feat(echo): add bounded S03 motion calibration probe`)

> **Evidence classification:** All numerical results below are reported from terminal diagnostics during the 22–23 November 2026 S03 investigation. Static USD information was additionally verified in a standalone, headless Isaac Sim 6.2 USD inspection. Measurements are *not* claims of hardware Intel RealSense D455 performance. All calibration adjustments in this report were performed in memory and were **not** applied to the production ROS pipeline.

## 1. Objective and boundaries

Validate that four independent RGB-D streams are usable together, that their calibration and TF frames are internally consistent, and that their reconstructed 3D surfaces agree across viewing directions. Keep the verified 3D LiDAR and navigation baselines intact for later controlled comparisons. This is **not** an assessment of cross-visit global relocalization, long-route map accuracy, dynamic-object tracking or absolute geometric error against independently sampled USD geometry.

## 2. Source components and launch contract

- Active static four-camera scene: `scenes/s01_d455/s01_d455_four_rgbd_raw_trial.usda`, composed from the S01 USD layers over the original Nova Carter warehouse scene.
- Dynamic experiment entry: `python3 scripts/start_echo_dynamic_four_d455.py`; manually press **Play** in Isaac Sim. The supervisor manages Isaac Sim, camera helpers, point-cloud conversion and S03 RGB-D synchronization. Stop with **Ctrl+C** in the supervisor terminal.
- Four D455-style assets reference `Isaac/Sensors/Intel/RealSense/rsd455.usd` from the Isaac Sim 6.2 NVIDIA content URL.
- Color and pseudo-depth are separate USD camera prims: `Camera_OmniVision_OV9782_Color` and `Camera_Pseudo_Depth`.
- Four depth CameraInfo relays (`scripts/echo_d455_{front,rear,left,right}_depth_camera_info.py`) clone the corresponding RGB CameraInfo and substitute the depth image header. Thus RGB/depth equality in published `K`, `P`, `R` and `D` is **by construction**, not an independent depth-intrinsic calibration.
- Standalone XYZ point clouds use `depth_image_proc/point_cloud_xyz_node`, published to `/echo/cameras/{front,rear,left,right}/depth/points`.
- The four-view synchronized RGB-D aggregate publishes `/echo/s03/rgbd_images` (`rtabmap_msgs/msg/RGBDImages`). Earlier S03 validation confirmed all four images in the ordered aggregate; the present geometric tests were separate, first-available point-cloud captures, **not timestamp-synchronized across cameras**.

## 3. Camera extrinsics and composed USD transforms

The strongest authored mount positions in the four-camera USD layer chain were found in the front fit / rear Hawk replacement / four-centered override layers. An independent **composed-stage** inspection of the actual depth camera prims returned the following positions relative to `chassis_link`:

| Camera | Composed depth-camera position (m) | Forward in chassis | Up in chassis |
|---|---|---|---|
| Front | `(0.1203, -0.000002, 0.3439)` | `+X` | `+Z` |
| Rear | `(-0.5837, 0, 0.3439)` | `-X` | `+Z` |
| Left | `(-0.2317, +0.1710, 0.3439)` | `+Y` | `+Z` |
| Right | `(-0.2317, -0.1710, 0.3439)` | `-Y` | `+Z` |

The referenced asset's color camera is offset by **11.5 mm** laterally from its pseudo-depth camera; internal orientations are essentially identical. A live ROS TF audit (`base_link` to each color optical frame) resolved all four transforms:

| Camera | Live color optical XYZ in `base_link` (m) | Optical forward yaw |
|---|---|---:|
| Front | `(0.1203, -0.0115, 0.3439)` | `0°` |
| Rear | `(-0.5837, +0.0115, 0.3439)` | `180°` |
| Left | `(-0.2202, +0.1710, 0.3439)` | `+90°` |
| Right | `(-0.2432, -0.1710, 0.3439)` | `-90°` |

All four live ROS optical +Y axes pointed vertically down in `base_link`, with a reported **0.000000°** deviation. These values agree with the camera calibrations decoded from retained nodes in the S03 dynamic RTAB-Map database, accounting for the 11.5 mm color offset and mount yaw. Across two retained observations, the largest decoded extrinsic translation variation was reported as **0.0201 mm**. This establishes internal calibration consistency, not independent world-geometry accuracy.

## 4. Intrinsics and point-cloud stream audit

Each of the eight live RGB/depth CameraInfo messages reported:

- Image resolution: **640 × 480**.
- `K`: `fx = fy = 317.04312`, `cx = 320`, `cy = 240`; zero skew.
- `P`: same `fx`, `fy`, `cx`, `cy` with zero translation terms.
- `R = I`; five zero `plumb_bob` distortion coefficients; zero binning and empty ROI.
- `K - P` principal-point difference: **0 px** for both axes.

All four standalone `PointCloud2` publishers were present and each delivered a **640 × 480**, 307,200-slot XYZ (`float32`) message. Each sample occupied 4,915,200 bytes. That number is an organized point-slot/payload count, **not** the count of valid points within a useful depth range. In the earlier stride-4 geometry audit, only approximately 8,960–9,072 sampled points per camera fell in the selected 0.25–8 m depth window. All four frames resolved through ROS TF before the successful geometry audit. An initial TF lookup failure was due to starting evaluation before the diagnostic TF listener had received its frames, not a validated camera malfunction.

## 5. Initial floor-plane geometry test

After transforming sampled valid depth points into `base_link`, near-floor samples were fit to `z = a x + b y + c`. The following earlier measurements used a preliminary near-floor selection and least-squares fitting:

| Camera | `a` (X slope) | `b` (Y slope) | Tilt magnitude | Median near-floor Z |
|---|---:|---:|---:|---:|
| Front | `-0.0000032` | `+0.0000007` | `0.00019°` | `-0.195 mm` |
| Rear | `-0.0031578` | ~`0` | `0.18093°` | `+3.625 mm` |
| Left | `-0.0015802` | `+0.0015752` | `0.12784°` | `+1.729 mm` |
| Right | `-0.0015803` | `-0.0015789` | `0.12799°` | `+1.715 mm` |

The residuals rounded to `0.000 mm` at the displayed precision, indicating exceptionally planar selected samples, **not exactly zero geometric error**. Near-floor selection presupposed a height range and is not independent floor segmentation. The per-camera surfaces did not initially agree on the same plane orientation.

## 6. In-memory half-pixel reprojection experiment

**Hypothesis:** The generated depth coordinates and ROS point-cloud reconstruction differ by approximately half a pixel in the image's vertical principal-point convention. This was tested **without changing live CameraInfo, USD assets, TF or RTAB-Map**.

For each cloud sample `(X,Y,Z)` in the optical frame, the experiment changed the effective `cy` by `delta` while keeping the captured depth `Z` fixed, using `Y_adjusted = Y - (delta / fy) × Z`; the adjusted XYZ was transformed into `base_link` and the near-floor plane refit. Trials covered `cy = 240.0`, `239.5`, `240.5` using the same captured cloud points and TF readout.

| Test principal point | Maximum cross-camera slope disagreement | Range of fitted plane intercepts |
|---|---:|---:|
| **240.0** (published) | `0.00157857` | `0.730897 mm` |
| **239.5** (in-memory) | **`0.00000194`** | **`0.000651 mm`** |
| **240.5** (in-memory) | `0.00315565` | `1.461709 mm` |

Changing `cy` to **239.5** reduced the maximum inter-camera slope disagreement by approximately **814×** and the range of plane intercepts by approximately **1,123×** in this one test. Corrected fitted slopes became approximately `a = -0.00157545`, `b = -0.00000018` in all four camera directions. The individual corrected intercepts were near `-0.002 mm`.

**Interpretation:** The nearly exact fit strongly supports a shared half-pixel projection-convention effect in the reconstructed point clouds. **It does not by itself prove the effective renderer principal point is 239.5**, nor demonstrate improved object-edge, wall, obstacle or map alignment accuracy. No permanent calibration correction has been approved or applied.

## 7. Residual common plane inclination and IMU evidence

After the in-memory half-pixel adjustment, the common near-floor slope magnitude was approximately **0.0903°** relative to `base_link`.

- `odom → base_link` TF predicted a nearly level frame: floor slopes `a = +0.00001123`, `b = +0.00000437` (inclination `0.000691°`). This planar odometry transform does **not** explain the observed common floor inclination.
- `/chassis/imu` provided a quaternion: `(x,y,z,w) = (0.000786840, -0.000003788, 0.999990761, 0.004224204)`. Under the assumed ROS orientation convention, derived pitch was approximately **−0.09017°**, close in magnitude to the corrected floor inclination.
- `base_link → chassis_imu` TF translation: **`(-0.21848, +0.02446, +0.15643) m`**. The mounting rotation was **roll = pitch = yaw = 0°**, confirming that the IMU's nominal axes align rotationally with `base_link`.

**Interpretation:** IMU pitch strongly supports a small real chassis-to-floor inclination while planar odometry does not represent it. This evidence is not time-synchronized with the captured point clouds and is not an independent simulator-physics ground-truth measurement; it does **not** establish the cause beyond doubt. In particular, a ROS frame may be intentionally planar while the physics chassis pitches slightly.

## 8. RTAB-Map context retained from the S03 dynamic trial

Preserve these raw, currently untracked outputs:

- `results/s03_dynamic_mapping/echo_dynamic_stationary_20261122T225017Z.db`
- `results/s03_dynamic_mapping/echo_dynamic_move_20261122T231445Z.db`
- `results/s03_four_mapping/echo_four_20261122T182854Z.db`
- `results/s03_four_mapping/echo_four_review.db`

The dynamic moving-trial DB included four camera calibrations in retained RTAB-Map nodes and recorded an approximately **0.1205 m** relative stored-node XY displacement between nodes 1 and 19. These are RTAB-Map graph/odometry estimates, **not independent simulator ground truth**. Rehearsal (`type=1`) bookkeeping links must not be mislabeled as independent loop closures. This calibration report does not re-evaluate mapping completeness, obstacle geometry, long-term relocalization or RTAB-Map map optimization.

## 9. Limitations and decision

**Passed / strongly supported:** Four-camera RGB-D/point-cloud availability; calibration message consistency; composed USD camera positions and level orientations relative to chassis; ROS TF extrinsics; retention of four camera models in RTAB-Map; excellent planar fit within selected low-height samples; strong inter-camera plane improvement under a half-pixel reprojection; agreement of residual plane inclination with IMU pitch magnitude.

**Open:** Absolute reconstruction accuracy against known USD surfaces and independently sampled physics ground truth; true effective RGB/depth projection at rendered pixel centers; generalization of `cy = 239.5` beyond sampled near-floor points and one stationary pose; synchronization between IMU and cloud samples; wall/obstacle/overlap alignment, motion/distortion and ghosting.

**Decision:** Preserve the baseline CameraInfo (`cy = 240`) and all current launch, TF and USD files. Do **not** silently apply the half-pixel adjustment. Treat `cy = 239.5` as a separate *candidate calibration variant* for controlled evaluation, not as a verified new default.

## 10. Recommended next research gates

1. **Independent geometry reference:** Compare the four camera clouds against known USD floor/wall planes and actual physics chassis pose at matched simulation timestamps. Test overlap consistency for stationary objects, not only the floor.
2. **A/B calibration experiment:** Keep original `cy = 240` and proposed `cy = 239.5` pipelines isolated. Use the same trajectory, camera frames, depth images and map parameters. Measure planar error, vertical-surface registration, geometric overlap and map consistency; record errors and runtime, with independent ground truth where feasible.
3. **S04 controlled sensor comparison:** Once camera geometry is bounded, compare one D455-style camera, four D455-style cameras and preserved 3D LiDAR on matched routes with clearly specified map/localization algorithms and independent pose/geometry ground truth.
4. **S05 and beyond:** Global relocalization on revisits; persistent object association and changed/moved/appeared/removed/occluded object reasoning. Dynamic tracking and Nav2 integration remain later work.

## 11. Repository safety checkpoint (23 Nov)

The user reported branch `echo/s01-d455`, HEAD `e7f83b4`, and a clean supervised shutdown (`[ECHO] Shutdown complete.`). Existing worktree entries included:

```text
 M config/rviz/echo_d455_front.rviz
 M config/rviz/echo_m04d_sensors.rviz
 M scenes/nav_baseline/navigation.usda
?? 1.5
?? base_link,
?? config/nav2/
?? results/s03_dynamic_mapping/
?? results/s03_four_mapping/
?? scripts/start_full_warehouse.sh
```

**Do not reset, clean, overwrite or broadly stage these files.** Keep raw `.db` results intact, and use explicit-path staging after reviewing any new report. The report itself was prepared from observed terminal evidence and does not imply a new Git commit or modification of the local ECHO checkout.
