# ECHO — Phase D closeout (S04): controlled mapping and sensor comparison

**Date:** 28 November 2026
**Development branch:** `echo/s04-sensor-comparison`
**Starting checkpoint:** `d0d746aee4caa5c91b0c4c8cccade6e542ddb471`
**Disposition:** experimental work complete at controlled engineering-pilot scope; Git release and evidence archive pending operator verification. Target tag `echo-phase-d-v1`. This is not a comprehensive sensor benchmark.

## 1. Objective and infrastructure

Compare one front D455-style RGB-D camera, four synchronized outward-facing D455-style cameras and the preserved 3D LiDAR. Use common simulator-derived odometry, known robot poses and recorded inputs. Keep NAV-0 independent of mapping. Ubuntu 24.04, Isaac Sim 6.2, ROS 2 Jazzy, RTX 5080, RTAB-Map core 0.23.14 / ROS 0.23.13; Nav2 1.3.13 (inventory). Phase C `echo-phase-c-v1` at `2983dda`, merged at `3ed6abe`, remains unchanged.

The supervisor `scripts/start_echo_dynamic_four_d455.py` now supports `--agents none|dynamic` (dynamic remains default), preserving calibration defaults and the original scene. Opt-in `--calibration coupled_xy` uses 640x480 registered RGB-D, fx/fy 317.04311998, cx/cy 319.5/239.5; `ECHO_S03_DEPTH_CY_MODE=baseline` disables the old depth-only experiment. `/echo/s03/rgbd_images` is `rtabmap_msgs/msg/RGBDImages`, ordered front/rear/left/right; `/front_3d_lidar/lidar_points`, `/chassis/odom`, `/clock`, `/tf`, `/tf_static` are recorded. NAV-0 consumes front-sector `/scan`, uses DWB/NavFn and odom rolling costmaps. The mapper does not control navigation or publish TF in these experiments.

Added optional session-layer obstacle fixture (`tools/s04/obstacle_fixture.py`) and passive chassis/actor pose logger (`tools/s04/pose_logger.py`), with bounded IRA startup wait and time-reset detection. No persistent USD edits; actor-free and dynamic launches exercised. Preserve original NAV-0 configuration; test-specific no-recovery BT and parameter copies are evidence, not replacements. Acquisition helpers retained under `tools/s04/`; default experimental config references the historical Downloads directory; use `--config` with a local copy whose BT path points to an existing file when relocating evidence. Do not run these live tools unattended. Simulator and Nav2 must be closed before offline processing.

## 2. S04-1: bounded NAV-0 qualification

- Clear path: a supervised 0.40 m goal completed, ~0.307 m net odometry displacement, final zero commands. The 0.10 m goal tolerance explains why goal distance is not achieved travel distance.
- Static obstacle: 0.20 x 0.60 x 0.90 m session-layer collider, nominal near face 0.60 m ahead of chassis origin, estimated initial body clearance 0.48 m. V2 trial `echo_s04_1b1_auto_20261128_001556_mqbkeekr` demonstrated Collision Monitor StopZone intervention while upstream nonzero commands continued. Archived goal log records watchdog cancellation; Nav2 log confirms goal canceled. Runner itself reported ABORT because it did not confirm the action result before exiting; retain this outcome, not a clean automated PASS. Measured projected minimum clearance about 0.265 m (historical body envelope, not a contact sensor). This qualifies the observed forward stop, not the runner's action-result gate. Earlier callback, idle-creep and simulation-reset failures remain in evidence.
- Stale monitor input: `echo_s04_1c_stale_20261128_002544_gify9cit`, isolated relay `/echo/s04/monitor_scan`, original `/scan` continued feeding costmaps. Last delivered stamp 4.300 s, gate closed 4.416667 s, first zero 5.816667 s; source age 1.516667 s versus configured 1.5 s. Driver recorded 9 nonzero upstream commands and 3 raw scans after stop, final motion blocked, canceled action, 0.1914 m odom displacement. Not a complete LiDAR outage test.

Collision protection is front-sector only. These supervised simulation cases do not certify physical safety, rear/turn protection or hardware stopping distance. Never remove Collision Monitor to force a goal. Pause only for immediate safety; a time reset invalidates a run. Keep abort/cancel path available.

## 3. S04-2: acquisition

| Dataset (all on 28 Nov) | Completed goals | Bag (GiB) | Recorded RGB-D aggregates / LiDAR | Transport loss reported | Matched pairs |
|---|---:|---:|---:|---:|---:|
| `echo_s04_2_mapping_pilot_20261128_003031_o1lt9s02` | 3/3 | 6.2 | 748 / 330 | 180 | 151 |
| `echo_s04_2_mapping_extended_20261128_004145_sh0_qi89` | 3/5 | 16.7 | 2020 / 915 | 347 | 269 |
| `echo_s04_4_mapping_dynamic_20261128_095513_j90q0ei6` | 3/3 | 5.2 | 622 / 332 | 162 | 149 |

Short route start-relative goals (metres, degrees): (0.60,0,0), (1.10,0.35,+35), (1.60,0,-35). Static 30.25 sim s (2.716667–32.966667); dynamic 29.833333 sim s (4.233333–34.066667). Measured paths approximately 1.82 and 1.78 m respectively; prior path comparison ~2 cm RMS nearest-path separation. Both completed the intended turns; this is not a complete closed-loop route. Extended route 2/4/6 m forward goals succeeded, fourth 90-degree turn failed to make progress; fifth not reached. Cause of underlying controller failure unverified (wall proximity plausible, costmaps/trajectory scores not captured). Restrict extended reconstruction to completed prefix 7.40–61.35 sim s. Do not describe intended return leg as measured. Approximately 5.9 m forward extent. Failed dynamic startup `echo_s04_4_mapping_dynamic_20261128_095120_c9s6vi3e`: runtime manager initially empty; no motion goals; fixed bounded initialization wait, then successful V2. Dynamic trial retained all four agent identities, 10 Hz target GT; no recorded Collision Monitor events in the three mapping recordings.

## 4. S04-3: matched-input geometric reconstruction

Direct projection, NOT SLAM optimization. One matched LiDAR/RGB-D group per 0.2 s bin, nearest unused RGB-D aggregate within 0.10 s, each camera/cloud projected at its own sensor stamp using recorded TF and interpolated odometry. Euclidean sensor range 0.5–10 m, camera pixel stride 4, LiDAR stride 1, common odom frame, voxel edge 0.05 m. Same front-camera samples in one/four modes. Maximum aggregate skew 50 ms; no rejected metadata in the three audits; matched-bin coverage 99.34%, 99.63%, 99.33%; maximum inter-pair gaps 0.2, 0.5, 0.3 s. Recorder transport loss still matters; coverage is matched schedule coverage, not all original messages delivered.

| Geometric occupied voxels | One D455 | Four D455s | 3D LiDAR |
|---|---:|---:|---:|
| Static short | 77,720 | 148,006 | 104,414 |
| Static extended prefix | 69,188 | 152,201 | 158,999 |
| Dynamic short | 73,767 | 146,903 | 111,476 |

Four vs one: +90.4% short, +120.0% extended. Four vs LiDAR: +41.7% short, -4.3% extended. These are sampled occupied surface voxels, not free space, independent accuracy or completeness against a complete scene mesh. Much of the gain is floor coverage.

| Height band (m) | Short one / four / LiDAR | Extended one / four / LiDAR |
|---|---|---|
| Below 0.1 | 49,725 / 115,534 / 85,151 | 31,718 / 103,898 / 129,949 |
| 0.1–1.5 | 7,653 / 8,849 / 8,843 | 9,274 / 11,938 / 15,599 |
| 1.5–2.8 | 7,132 / 8,083 / 10,420 | 8,450 / 11,020 / 13,451 |
| >=2.8 | 13,210 / 15,540 / 0 | 19,746 / 25,345 / 0 |

In the extended map, above-floor totals are 37,470 / 48,303 / 29,050. Four cameras add elevated geometry but LiDAR has more voxels within the shared lower height band (0.1–2.8 m). Directional nearest-neighbor comparison of voxel centers restricted to that height band, numerical tolerance 1 micrometre at 10 cm threshold:

| Directed comparison | Short within 10 cm | Extended within 10 cm |
|---|---:|---:|
| One -> LiDAR | 96.18% | 97.15% |
| LiDAR -> one | 98.13% | 91.58% |
| Four -> LiDAR | 95.92% | 96.82% |
| LiDAR -> four | 98.76% | 99.87% |

Nearest-neighbor metrics depend on sampled coverage and quantization; they are not independent depth accuracy or verified mutual visibility. All single-camera voxels are included in four-camera maps. Short/extended GT–odom initial-alignment XY RMSE 1.240/1.648 mm (303/540 poses). Simulator odometry describes the same physical simulation chassis; not independent wheel odometry. No new physical D455 calibration or accuracy claim.

## 5. S04-3: RTAB-Map pipeline replay and native exports

Use original short static bag, isolated ROS domain, recorded `/clock`, no motion topics, half-speed wall replay, separate DBs and common external odometry. Initial camera replay missed 232/748 front relay messages and four-camera PnP reported missing OpenGV support. Camera V2 uses reliable aggregate delivery and `Vis/EstimationType=0` (3D-to-3D), `Reg/Strategy=0`; both audits received all 748 aggregates, both DBs have 32 observations / 22 retained nodes; no unsupported PnP errors. The LiDAR pilot uses ICP (`Reg/Strategy=1`) and cloud input, separate native schedule/QoS. All SQLite integrity checks passed. `publish_tf=false`. Full audit delivery does not imply internal acceptance of every message. LiDAR is sensor-plus-pipeline comparison, not pure visual sensor ablation.

| Metric | One D455 V2 | Four D455 V2 | LiDAR pilot |
|---|---:|---:|---:|
| Stored / retained nodes | 32 / 22 | 32 / 22 | 30 / 24 |
| DB MiB | 20.66 | 81.78 | 16.52 |
| Mean logged processing ms | 31.00 | 77.78 | 49.45 |
| Median logged processing ms | 29.15 | 73.85 | 29.1 |
| Distinct global closure links | 0 | 1 | 0 |
| Distinct local spatial links | 1 | 6 | 4 |
| Native export odom / optimized points | 74,284 / 84,990 | 126,833 / 138,075 | 77,083 / 87,957 |
| Odom-only XY RMSE (mm) | 0.062 | 0.062 | 0.126 |
| Optimized XY RMSE (mm) | 18.89 | 6.60 | 16.84 |
| Maximum optimized XY error (mm) | 27.85 | 10.34 | 32.13 |
| Optimized yaw RMSE (deg) | 0.046 | 0.037 | 0.175 |

Export via `rtabmap-export --cloud --poses --poses_format 11 --opt 0` (optimized) and `--opt 3` (odometry), `--voxel 0.05 --min_range 0.5 --max_range 10`, camera `--decimation 4`, LiDAR `--scan --decimation 1`, copies of DBs. Exact logs/commands retained. GT linearly interpolated to each native retained-node stamp, initial planar alignment only (no fitted trajectory). The included read-only audit recalculates metrics; tiny differences from earlier conversation (<0.003 mm RMSE) reflect planar alignment implementation/rounding. Both camera modes have identical retained timestamps. Four-camera optimization has ~65% lower XY error than single-camera optimization, but all three optimizations worsen the already very consistent simulator odometry. Added closure links are not proof of correct global relocalization; more map voxels can reflect pose changes, not additional observations. Processing time is per logged mapper iteration, not total CPU/GPU/RAM or real-time end-to-end performance. Four vs one processing ~2.51x, storage ~3.96x. Do not conflate native exports with matched-input direct geometry.

## 6. S04-4: dynamic contamination pilot

Actor-free short static reference vs same nominal route with two moving red Carters and two workers; measured paths close but not identical. Both workers came within evaluated range; both NPC Carters remained farther than 10 m. Timestamp-interpolated actor-root cylinders in world coordinates, aligned once to odom: worker radius 0.6 m, z offset 0.1–2.1 m; robot radius 1 m, z offset 0.1–1.6 m. No exact segmentation, visibility or ground-truth ghost labels. Candidate static comparison uses same-sensor static map, 10 cm nearest-neighbor threshold; differing route, visibility and sampling also influence results. Union of voxel keys across actors, not sum of overlapping voxels. Recomputed from actual PLY and audit JSON.

| Candidate evidence | One D455 | Four D455s | 3D LiDAR |
|---|---:|---:|---:|
| Worker-proximity samples | 2,500 | 10,288 | 20,408 |
| Unique candidate voxels | 1,643 | 6,364 | 6,906 |
| Fraction of complete map | 2.23% | 4.33% | 6.20% |
| >10 cm from corresponding static map | 1,637 | 6,075 | 6,731 |

Front 2,500; left 4,029; right 3,759; rear zero candidate point samples. Side cameras contribute 75.7% of camera candidate returns; one front camera captures one worker, four cameras capture candidates near both. No candidate returns from the NPC robots. More observed moving geometry motivates temporal filtering, not a sensor failure. This does not isolate stationary vs moving versions of identical actors and does not establish exact dynamic-object detection performance.

## 7. Disposition, limitations and S05 decision

S04-0 inventory complete; S04-1 bounded stop/stale response qualified with obstacle runner outcome caveat; S04-2 short route and extended prefix complete at pilot scope (full return leg not achieved); S04-3 static geometric and native RTAB-Map comparison complete at pilot scope; S04-4 human-proximity contamination pilot complete. S04-5 requires verified archive and Git checkpoint. Deferred: repeated trials/statistical confidence, complete revisit loops, true cross-camera temporal correspondence/absolute scene-surface accuracy, stationary-vs-moving matched actors, nearby NPC contamination, dynamic RTAB-Map optimization and CPU/GPU/RAM profiling. These are explicit scope reductions, not passed tests. No additional Phase D live experiment required for the accepted engineering-pilot closure. Preserve both failed and successful evidence.

Phase E / S05: primary candidate is four-camera RTAB-Map with 3D-to-3D registration; single RGB-D and LiDAR are comparison baselines. Use existing short static DB for sanity checks, not a validated global reference map. Establish adequate static reference coverage, freeze reference DB copies, validate known-pose then independent unknown-start localization with no GT initialization, no accidental odometry continuity or map growth. Measure correct-localization time, success/false acceptance, position/yaw error under independent GT, viewpoint and occlusion. Keep Nav2 baseline independent until later integration. No calibration reopening without blocker. Phase G is the candidate original research contribution: joint multi-view instance association, localization uncertainty, visibility/negative evidence and multi-visit hypothesis management; novelty not yet established. F controlled changes precede G; tracking/prediction and ECHO Nav2 are H/I extensions.

## 8. Evidence and reproducibility

Historical working paths were `~/Downloads/<trial>`; the closeout archive is planned at `~/echo_archive/phase_d_20261128/downloads/` (copies, not deletion). `reviewed_summaries/` contains the exact uploaded summary archives and SHA-256 manifest. `ECHO_S04_EVIDENCE_SHA256_20261128.txt` hashes original reviewed archives; local raw archive gets its own full `SHA256SUMS` and source/destination verification. Retain `echo_s04_rtabmap_camera_v2_1rn44u0k/{one_d455,four_d455}/map.db` and `echo_s04_rtabmap_pilot_ihns4ynb/lidar/map.db`. Retain replay V1 camera DBs as superseded evidence. `echo_s04_rtabmap_export_Im2WNU` contains native exports and copies of DBs. Do not commit raw bags, DBs or bulky map ZIPs. Git contains tools, small metrics, docs and evidence hashes only.

Reproduce geometric metrics with `python3 tools/s04/echo_s04_audit_map_exports.py <archive>/reviewed_summaries` (NumPy and SciPy; no ROS needed). Result JSON in `docs/ECHO_S04_MAP_METRICS_20261128.json`. This audit was run against the actual uploaded archives. Sensor projection scripts require sourced Jazzy and recorded message types. Source revisions retained unchanged in `tools/s04/` (see handoff for commands); runner paths assume `~/echo`, archive paths may be passed explicitly. `echo_s04_run_obstacle_v2.py` also serves as a shared helper; its historical action-result caveat remains. For preview on system Python use `python3 -I` via `/usr/bin/python3` to avoid NumPy/Matplotlib ABI issue. Archive and Git state remain operator-verified steps, not operations performed on user's workstation by this report.
