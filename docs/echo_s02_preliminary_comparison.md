# ECHO — S02 Preliminary Front RGB-D / 3D LiDAR Mapping Study

**Experiment:** 19 November 2026, previous development session “1- 4 D455 Cams”  
**Report prepared:** 20 November 2026, from prior-session results and the current documentation audit.  
**Evidence level:** historical **reported** experiment, **not** a new raw-data reanalysis.  
**Code state:** `echo/s01-d455` HEAD `5b40f2b` (front S01 only). The S02 mapping launches, reconstruction and trial scripts were **untracked** in the user-reported Git status on 20 Nov; do not state that a fresh clone reproduces S02 yet.

## 1. Objective

Produce an initial 3D map from the **single front simulated D455-style RGB-D camera** through RTAB-Map, and compare selected reconstructed geometry against the preserved Nova Carter 3D LiDAR over a shared short route. Establish data collection and analysis tools, not a final sensor ranking or a multi-camera localization result.

## 2. Environment and available code

- Ubuntu 24.04, Isaac Sim 6.2, ROS 2 Jazzy, RTAB-Map.
- White Nova Carter in warehouse, front D455 RGB/depth/CameraInfo/TF from S01 and existing `/chassis/odom`.
- Existing 3D LiDAR retained; historical ROS point topic `/front_3d_lidar/lidar_points`.
- Earlier front integration reference: `docs/echo_s01_front_d455.md` and `scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda`.

S02 files **confirmed present on disk by user terminal**, but **not included as tracked source contents** in the uploaded audit ZIP:

```text
launch/echo_s02_front_mapping.launch.xml
launch/echo_s02_front_mapping_range10.launch.xml
tools/echo_s02_multiview_trial.py
tools/echo_s02_multiview_trial_heading_v1.py
tools/echo_s02_dual_sensor_trial.sh
tools/echo_s02_crop_lidar_to_d455_fov.py
tools/echo_s02_reconstruct_lidar.py
tools/echo_s02_replay_odom_path.py
```

Previous entry points as reported (inspect arguments and code before rerunning):

```bash
cd ~/echo
python3 tools/echo_s02_multiview_trial.py --execute
bash tools/echo_s02_dual_sensor_trial.sh
# RTAB-Map configuration existed at:
# launch/echo_s02_front_mapping_range10.launch.xml
```

**Local data location reported previously** (verify actual presence and manifest, never commit large binaries blindly):

```text
diagnostics/s02/trial_20261119_230409/rtabmap.db
# Other run-specific paths within diagnostics/s02/:
d455_map_same_trial.ply
lidar_map_odom_10m_0.05m_stride5.ply
overlay_d455_blue_lidar_orange.ply
lidar_bag/
```

The ZIP contained documentation/file trees only: we did not inspect the S02 database, MCAP/bag samples, PLY clouds, scripts or numerical result files in this new conversation.

## 3. Previously reported results

| Measurement | Prior-session result |
|---|---:|
| Earlier front RGB-D map observations | 65 |
| Earlier map graph links | 54 |
| Earlier odometry travel | approximately 1.19 m |
| Common-route dual-sensor RTAB-Map nodes | 59 |
| Common-route LiDAR scans | 720 |
| Common-route traveled distance | approximately 1.18 m |
| Exported RGB-D cloud size | approximately 311,351 points |
| Exported LiDAR cloud size | approximately 89,101 points |
| D455 query points in FOV-restricted NN comparison | 49,784 |
| FOV-cropped LiDAR reference points | 37,324 |
| Median D455→LiDAR nearest-neighbor distance | **0.0643 m** |
| 90th percentile D455→LiDAR NN distance | **0.2830 m** |
| D455 query points within 0.10 m of LiDAR NN | **70.52%** |
| D455 query points within 0.20 m of LiDAR NN | **85.31%** |

The previous session additionally reported that sampled RTAB-Map poses corresponded closely to supplied `/chassis/odom`. This is **consistency with an external odometry input**; it does not constitute an independent global-pose accuracy result or unknown-start localization success.

## 4. Interpretation: what is and is not established

**Supported at the historical-report level:** A working front-D455 RTAB-Map mapping workflow was exercised, a 3D LiDAR recording was obtained on a shared route, and a preliminary cloud correspondence visualization/nearest-neighbor analysis was produced.

**Not established by this run:**

- Four D455 streams, a four-input RTAB-Map instance or an integrated four-camera map.
- Global relocalization from a genuinely unknown pose, loop-closure reliability or independent camera-derived odometry quality.
- True D455 geometric depth accuracy, 3D LiDAR accuracy or superiority of either modality.
- Long-term route/scene coverage, repeatability across seeds, global map consistency, computational scaling or physical-device performance.

### Why the cloud distances are not sensor accuracy

Cloud-to-cloud NN distances combine real surface mismatch, incomplete/occluded geometry, sampled ray locations, point density, voxelization, map/odom transforms, timing, image registration and FOV approximations. A cropped LiDAR cloud is **not** a perfect representation of the surfaces visible to each D455 pixel. To evaluate sensor geometry, use an independently known scene surface/GT model and carefully specified co-visibility masks; preserve each sensor's physical/virtual FOV limitations.

The common route was short (~1.18 m). All reported values must be reopened and verified from the actual outputs before being used as scientific accuracy claims or a published statistical comparison.

## 5. Reproduction and audit checklist (not yet executed in this chat)

- Record `git status --short`, review the untracked S02 files and save a reviewed, explicit commit without touching unrelated local changes.
- Record Isaac Sim/ROS/RTAB-Map versions and all D455/LiDAR/odom topic names, QoS and TF names.
- Confirm which scene, launch args, code versions and random/trajectory settings generated each trial.
- Verify `rtabmap.db` opens and contains the expected nodes/links with valid sensor data; reconstruct the map with a reproducible script.
- Verify the bag/PLY inputs and timestamp overlap actually come from the same run.
- Inspect crop, voxelization, nearest-neighbor direction, coordinate frames, invalid-depth filtering and handling of occlusions.
- Recompute summary tables/figures from raw inputs and add direct provenance (checksums/manifests/logs) rather than transcribing figures.
- Keep large raw MCAP/PLY/DB data outside Git unless deliberately publishing a dataset.

## 6. Next steps

1. Preserve the S01/S02 baseline and untracked scripts; no repeat of this preliminary comparison is required before adding new cameras.
2. Audit the actual USD front-camera scene/Action Graph and helper scripts.
3. Add rear, then left/right calibrated RGB-D assemblies in an isolated derived scene, while retaining 3D LiDAR.
4. Confirm one RTAB-Map process consumes all four camera views into one map/database.
5. Design and run properly matched sensor mapping and unknown-pose relocalization studies under `docs/ECHO_EXPERIMENT_PROTOCOL.md`.

## 7. Related records

- `docs/echo_s01_front_d455.md` — S01 sensor integration, measured synthetic intrinsics/TF, observed depth and synchronization, RViz and QoS lessons.
- `docs/echo_phase_b_m04c_m04d.md` — earlier 3D LiDAR/worker visibility experimental baseline.
- `docs/ECHO_MASTER_PLAN.md` — project goals, stages and validation gates.
- `docs/ECHO_EXPERIMENT_PROTOCOL.md` — future GT-referenced comparison methodology.
