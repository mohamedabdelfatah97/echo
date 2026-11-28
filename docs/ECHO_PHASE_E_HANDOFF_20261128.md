# New chat handoff — ECHO Phase E / S05

Continue ECHO from the Phase D controlled mapping and sensor comparison chat. First read `ECHO_PROJECT_CONTEXT_20261128.md`, `ECHO_ROADMAP_20261128.md`, `ECHO_PHASE_D_CLOSEOUT_20261128.md` and actual repository `~/echo`. The experimental work of Phase D is closed at engineering-pilot scope; verify the archive, release tag `echo-phase-d-v1`, branch/main status from terminal output before beginning. Do not assume Git actions succeeded. If closure verified, create `echo/s05-global-relocalization` from the verified Phase D main checkpoint (proposed branch name). No Phase C recalibration or Phase D rerun unless reproducible blocker. No object memory or tracking implementation yet.

Environment Ubuntu 24.04, Isaac Sim 6.2, ROS 2 Jazzy, RTX 5080. One-terminal automation preferred, no Script Editor. Always state Isaac Sim CLOSED/NOT PLAYING/PLAYING. Inspect actual files, USD and interfaces before edits. Preserve baseline, uncommitted work, raw evidence. New diagnostic ZIPs directly in `~/Downloads`; archive copies at `~/echo_archive/phase_d_20261128/downloads`, exact uploaded evidence in sibling `reviewed_summaries` (verify). Raw data should remain local. Historical Downloads paths are recorded in logs; adjust explicit config/DB paths if archived. Do not blindly replay a stale absolute path or overwrite DBs.

## First response expected

Briefly state verified Phase D results and scope limits, then provide one read-only inventory block for S05-0: Git/status/tag, archive check, available RTAB-Map executables and ROS package interfaces, existing static map DBs/parameters, map/odom TF ownership and installed localization parameters. Do not launch live motion immediately. Agree on minimum reference map coverage, unknown-start protocol and metrics before new experiment.

## Starting artifacts and methods

- Static short bag `echo_s04_2_mapping_pilot_20261128_003031_o1lt9s02/bag`, completed 3-goal zigzag ~1.82 m; extended `echo_s04_2_mapping_extended_20261128_004145_sh0_qi89/bag` completed first three forward goals only (7.4–61.35 sim s). These are small mapping pilots, not full reference maps. No full loop/revisit proof.
- Camera DBs `echo_s04_rtabmap_camera_v2_1rn44u0k/{one_d455,four_d455}/map.db`; LiDAR DB `echo_s04_rtabmap_pilot_ihns4ynb/lidar/map.db`. Exact params.yaml/command.json and audit/logs in same per-sensor directories. Four-camera `Vis/EstimationType=0`, `Reg/Strategy=0`, `rgbd_cameras=0`, `subscribe_rgbd=true`, `publish_tf=false`; reliable offline QoS vs BEST_EFFORT live aggregate. Prior multi-camera PnP rejected without OpenGV; 3D-to-3D V2 accepted links without error. Both camera DBs 32 observations / 22 retained; LiDAR ICP 30/24. These are reference artifacts for sanity checks, not validated global localization. Freeze original, use new working copies for experiments.
- Static geometry and native exports in reviewed summary archives. Reconstruct scripts in `tools/s04/`; `echo_s04_audit_map_exports.py <archive>/reviewed_summaries` recomputes comparison without ROS. Native optimization XY RMSE one/four/LiDAR 18.89/6.60/16.84 mm, worse than simulator-derived odom. Additional graph links do not demonstrate global localization.
- Dynamic dataset `echo_s04_4_mapping_dynamic_20261128_095513_j90q0ei6`; two workers observed by side/front views, red Carters out of range; candidate contaminated voxels not segmented ground truth. Not required to rerun for S05.

## Environment and supervisor reference

```bash
cd ~/echo
source /opt/ros/jazzy/setup.bash
source ~/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ECHO_S03_DEPTH_CY_MODE=baseline
python3 scripts/start_echo_dynamic_four_d455.py --agents none --calibration coupled_xy --check
```

This is read-only preflight, not a launch. Use `--agents none` for static reference; dynamic default still available. Live stream `/echo/s03/rgbd_images`, front/rear/left/right registered 640x480, corrected K/P principal point. `/chassis/odom` and `odom -> base_link`, LiDAR `/front_3d_lidar/lidar_points`. NAV-0 baseline separate. If using S04 runners, `--config` points at a fresh copy of the archived no-recovery parameters with its BT path adjusted; do not change baseline. Inspect source before running. All motion supervised, no reverse/turn safety assumption. Pause for safety only and abort on time reset.

## S05 acceptance direction

Build adequate reference map for revisits, immutable DB and map coordinate/GT transform. Localization-only configuration must not grow the reference map; inspect installed parameter behavior (e.g. Mem/IncrementalMemory) and topic/action APIs before commands. Known-pose sanity then unknown-start trials with reset independent odometry/no GT initialization; score time to correct pose, false accepts/fails, XY/yaw error against independent scoring GT, confidence validity, opposite-facing viewpoints and occlusion. Specify success threshold and confirmation window before tests. Compare single/four RGB-D and LiDAR where comparable, note registration differences. No live Nav2 integration until localization separately qualified. Later Phase G joint uncertainty/visibility/evidence-based persistent object memory is candidate novelty, not already implemented or proven.
