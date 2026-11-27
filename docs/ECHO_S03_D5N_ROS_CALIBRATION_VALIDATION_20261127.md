# ECHO S03-D5N — ROS coupled XY calibration validation

Date: 27 November 2026. Starting checkpoint: `ea00c4d` (`echo/s01-d455`).

## D5N-A: archived ground-truth replay (passed)

Actual ROS Jazzy `depth_image_proc` nodes processed identical input images under baseline and coupled XY CameraInfo. Fixed focal lengths 317.04311998 px, 640 x 480, unchanged TF. Input D4F pose trace and D4E native ROS frames; D5M nine-sphere capture. Output `echo_s03_d5n_ros_gt_replay_20261127T163633Z.zip` includes report, actual ROS cloud arrays and replay source.

| Test | Baseline RMSE mm | Coupled XY RMSE mm |
|---|---:|---:|
| Front floor | 2.334592 | 0.0000924 |
| Rear floor | 2.797911 | 0.0001467 |
| Left floor | 2.937657 | 0.922786 |
| Right floor | 2.667092 | 0.0001005 |
| Front native sphere radial surface | 2.295572 | 1.676532 |
| Front registered sphere radial surface | 2.187444 | 1.691445 |

Floor metrics use simulation-manager pose time. 94 localized left-camera samples exceed 1 mm residual; keep them in official score. Excluding them gives approximately 0.000138 mm, a diagnostic only. Very small simulated planar residuals are not physical D455 accuracy. Sphere tests used common samples (native 1344, registered 1467). Registered arrays and valid masks were identical; corrected K/P propagated. Static archived replay does not validate SLAM pose accuracy or live throughput.

## D5N-B: live parallel candidate transport (passed)

Output `echo_s03_d5n_b_live_20261127T163940Z.zip`: 25.02 s measurement, original and isolated candidate topics, source and per-pair statistics. No relay or embedded-message validation errors. 176 baseline / 171 candidate aggregates, 7.02 / 6.85 Hz under concurrent compute load, median spread 16.67 ms and max 50 ms for both. 23 identical full timestamp sets. Within-camera RGB/depth timestamps match in validated aggregates. The rear register node issued nine initial TF lookup errors before recovering.

| Camera | Matched pairs | Common valid depth pixels | Mask disagreements |
|---|---:|---:|---:|
| Front | 73 | 22355520 | 0 |
| Rear | 75 | 22961383 | 0 |
| Left | 62 | 18981097 | 0 |
| Right | 67 | 20515244 | 0 |
| Total | 277 | 84813244 | 0 |

All compared valid depths agree exactly. Matched pairs require matching embedded RGB and registered-depth timestamps and identical RGB payload hashes. The archive stores statistics, not raw images; the totals are reproducible from per-pair records, not an independent raw-pixel replay. No production topics modified by D5N-B. No standalone speed, map or relocalization claim.

## Integration candidate (runtime validation pending)

`python3 scripts/start_echo_dynamic_four_d455.py --calibration coupled_xy` selects the correction (cx=319.5, cy=239.5) in both K and P for RGB and native depth. A separate supervised relay publishes under `/echo/s03/calibration/coupled_xy/{side}/{sensor}/camera_info`; source headers and raw images remain unchanged. The four native XYZ converters and S03 registration/RGBD sync nodes consume the selected root. Their original output names remain unchanged; raw `/echo/cameras/.../camera_info` remains the simulator baseline even in coupled mode. Any independent consumer of raw CameraInfo must explicitly select the corrected root if using that model. The default `--calibration baseline` uses original topics; legacy depth-only `ECHO_S03_DEPTH_CY_MODE=corrected` is rejected by this supervisor to avoid mixed models. Original helper code, USD, TF, Nav2 and LiDAR are unchanged except CameraInfo input remaps in rear/side cloud helper launch scripts. No duplicate synchronization pipeline required. Keep baseline and corrected RTAB-Map databases separate.

After restarting, run `python3 tools/echo_s03_d5n_selected_mode_audit.py --mode coupled_xy --seconds 20`. Check native cloud subscriptions and repeat in baseline mode before considering integration verified. This patch is source-checked, not yet tested on the live workstation. The next gates are D7 independent odometry ground truth and D8 NAV-0 + four-camera mapping, then Phase C closure. No final mapping-quality claim here.

## D5N-C — Integrated runtime validation

Validation date: 2026-11-27

Both selectable calibration modes passed runtime validation.

| Metric | Baseline | Coupled XY |
|---|---:|---:|
| Principal point cx | 320.0 | 319.5 |
| Principal point cy | 240.0 | 239.5 |
| Native cloud converters passed | 4/4 | 4/4 |
| Validated cloud frames | 12 | 12 |
| Sampled XYZ points | 230400 | 230400 |
| Valid aggregate messages | 185/185 | 170/170 |
| Aggregate arrival rate (Hz) | 9.4117 | 9.4297 |
| Maximum timestamp spread (ms) | 33.333334 | 50.0 |
| Worst P99 projection error (px) | 0.00002115 | 0.00002092 |
| Missing topics | 0 | 0 |
| Validation errors | 0 | 0 |

The initial integration exposed a native point-cloud CameraInfo
routing issue caused by relative topic remapping. This was fixed
using fully resolved CameraInfo topic names.

All four native converters were subsequently verified through
both live subscription inspection and numerical recovery of the
principal point from published XYZ coordinates.

Baseline rollback restored the original calibration and produced
no unexpected corrected CameraInfo publishers.

NAV-0 startup also passed with coupled XY calibration.

The default launcher mode remains baseline. Coupled XY is selected
explicitly for subsequent RGB-D mapping experiments.

These results validate the simulated camera geometry and ROS
integration, not physical Intel RealSense D455 sensor accuracy.

The diagnostic rates are not controlled throughput benchmarks.
Independent motion ground truth and combined navigation/mapping
validation remain pending.

D5N-C status: PASSED.
