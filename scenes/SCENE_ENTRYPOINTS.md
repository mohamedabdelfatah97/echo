# ECHO — Scene entry points (22 November 2026)

This is an entry-point index, **not** a list of independent USD assets. Keep the existing `scenes/s01_d455/*.usda` layers together because the four-camera scene composes through all 17 of them. Do not rename/move sublayers without recomputing their relative paths and revalidating Isaac Sim.

## S03: Four D455-style RGB-D cameras, static warehouse

- **Open** `scenes/s01_d455/s01_d455_four_rgbd_raw_trial.usda`
- **Start** `bash scripts/start_echo_d455_four_live.sh`
- In Isaac Sim: **File → Open** the scene above; then press **Play**.
- Start synchronized four-RGB-D pipeline separately with `ros2 launch ~/echo/launch/echo_s03_four_rgbd_sync_probe.launch.py` (after sourcing ROS Jazzy). This is a synchronization probe, not a complete reproducible mapping runner.
- S03 measurements: four populated RGB-D observations per combined message, ~6 Hz observed aggregation, one preliminary RTAB-Map database. Geometric accuracy, long-route SLAM and localization are **not validated**.
- **No dynamic NPCs are spawned by the four-camera startup script.**

## M04C: Main white ROS Carter + two red Carter NPCs + two animated warehouse workers

- **Start** `bash scripts/start_full_warehouse_two_humans.sh`.
- Python launcher: `scripts/start_full_warehouse_two_humans.py`.
- Agent config: `config/agents/ira/warehouse_two_npcs_two_humans.yaml`.
- Environment/robot USD: `scenes/full_warehouse/main_robot_ros.usd`.
- NPC config: `config/agents/ira/carter_npc_red.yaml`.
- Worker asset directory: `assets/characters/ira_warehouse_worker/` (verify locally when launching).
- IRA creates the NPC prims at **runtime**; the base USD alone is not the complete scenario. Historical M04C validation is in `docs/echo_phase_b_m04c_m04d.md`.
- This is **not yet** the combined M04C-plus-four-D455 scene.

## Other preserved baselines

- **Single front D455:** `scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda` (`scripts/start_echo_d455_front_stack.sh`)
- **Nav/LiDAR baseline:** `scenes/nav_baseline/navigation.usda` (`scripts/start_echo_sim.sh`) — contains local uncommitted changes.
- **Legacy full warehouse:** `scenes/full_warehouse/warehouse.usd` (`scripts/start_full_warehouse.sh`)
- **Single-worker M04D crossing:** `scripts/start_m04d_worker_crossing.sh`

## Preservation rules

Do not delete or rename `scenes/full_warehouse/*.usd` until their binary composition references have been audited with the USD APIs. Do not run `git clean`, `git reset --hard` or `git add -A` here. Keep raw S03 databases and PLY exports out of source-only commits. See `docs/ECHO_scene_cleanup_review_20261122.md` for the file-by-file audit decision.
