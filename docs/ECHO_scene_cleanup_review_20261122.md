# ECHO scene cleanup review — 22 November 2026

**Source:** `echo_scene_cleanup_audit_20261122T205335Z.zip`, generated read-only against `/home/m12/echo` on branch `echo/s01-d455`, commit `a8084e220966f5e065bb0ac67595483c883dde41`. This report is a classification of the uploaded snapshot, **not** a live inspection or a verified scene modification.

## Decision

**Delete or move zero USD scenes at this stage.** All 17 text USD layers in `scenes/s01_d455/` are on the recursive sublayer chain of the working four-camera scene. The remaining five `scenes/full_warehouse/*.usd` are opaque binary USD and must not be declared unused from a text scan; `main_robot_ros.usd` is used by both S01 and the dynamic launcher. `scenes/nav_baseline/navigation.usda` contains uncommitted changes and is used by the navigation launcher. Re-label and document entry points rather than flattening or moving their dependencies without runtime verification.

## Established entry points

| Purpose | Entry point | Status and warning |
| --- | --- | --- |
| Four-camera static mapping | `scenes/s01_d455/s01_d455_four_rgbd_raw_trial.usda` + `scripts/start_echo_d455_four_live.sh` | Four RGB-D streams, TF and S03 aggregation observed in prior trials. This launcher opens Isaac Sim but requires selecting the stage in the GUI. Does **not** spawn two red NPCs or two workers. |
| Known verified front D455 | `scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda` + `scripts/start_echo_d455_front_stack.sh` | Earlier one-camera baseline; a root inside the four-camera dependency chain. |
| Full warehouse with two red Carters + two workers | `scripts/start_full_warehouse_two_humans.sh` → `.py` → `config/agents/ira/warehouse_two_npcs_two_humans.yaml`, with `scenes/full_warehouse/main_robot_ros.usd` as environment | Config explicitly declares `robot.groups.echo_npc_carters.num: 2` and `character.groups.warehouse_worker.num: 2`. This is a **runtime-spawned IRA setup**, not a static four-D455 USD. Earlier M04C report records four unique identities. The audit did not re-run the launch. |
| Dynamic one-worker crossing | `scripts/start_m04d_worker_crossing.sh` and its config | Preserve experimental LiDAR baseline and GT tools. |
| Navigation baseline | `scripts/start_echo_sim.sh` → `scenes/nav_baseline/navigation.usda` | Existing file is modified; do not overwrite. |
| Historical full warehouse launcher | `scripts/start_full_warehouse.sh` → `scenes/full_warehouse/warehouse.usd` | **In use by a launcher**. Do not delete despite absence from the four-camera chain. |

**Important:** The two-workers dynamic scene is not shown by this audit to have four D455 cameras. Combining both setups is a separate future derived experiment.

## USD file inventory and classification

| File | Protect? | Why |
| --- | --- | --- |
| `scenes/full_warehouse/main_robot_ros.usd` | YES | Common underlying robot asset; referenced by S01 chain and dynamic IRA config. Binary USD. |
| `scenes/full_warehouse/warehouse.usd` | YES | Referenced by `scripts/start_full_warehouse.sh`; binary/opaque. |
| `scenes/full_warehouse/environment_layer.usd` | YES, pending USD introspection | Binary/opaque. Could be composition dependency or historical baseline. |
| `scenes/full_warehouse/agent_snapshot.usd` | YES, pending USD introspection | Binary/opaque snapshot; use cannot be excluded. |
| `scenes/full_warehouse/agent_snapshot_layer.usd` | YES, pending USD introspection | Binary/opaque snapshot layer; use cannot be excluded. |
| `scenes/nav_baseline/navigation.usda` | YES | Navigation launcher and uncommitted changes. Text-scanner reports a self-edge: verify scanner artifact rather than infer actual self-reference. |
| `scenes/s01_d455/s01_d455_four_rgbd_raw_trial.usda` | YES | Currently working four-camera entry point. |
| `scenes/s01_d455/s01_d455_four_centered_owl_off.usda` | YES | Sublayer of final four-camera scene. |
| `scenes/s01_d455/s01_d455_four_centered_probe.usda` | YES | Sublayer of final four-camera scene. |
| `scenes/s01_d455/s01_d455_four_mount_probe.usda` | YES | Sublayer of final four-camera scene. |
| `scenes/s01_d455/s01_d455_front_rear_hawk_replaced.usda` | YES | Sublayer of final four-camera scene. |
| `scenes/s01_d455/s01_d455_front_rear_ros.usda` | YES | Sublayer of final four-camera scene. |
| `scenes/s01_d455/s01_d455_front_rear_mount_probe.usda` | YES | Sublayer of final four-camera scene. |
| `scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda` | YES | Front verified baseline and dependency. |
| `scenes/s01_d455/s01_d455_front_ros_units_probe.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front_ros_rgb_info_verified.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front_ros_rgb_verified.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front_ros_rgb.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front_fit_v2.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front_25mm_out.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front_centered.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front_hawk_off.usda` | YES | Sublayer. |
| `scenes/s01_d455/s01_d455_front.usda` | YES | Lowest S01 text layer, referencing the main robot USD. |

No missing USD dependencies were found **among the parsed text USD sublayer paths**; binary composition arcs and external assets were not fully audited. The audit also marked `scenes/echo_warehouse_nav_reference.usda` **missing as a requested root**, despite no parsed missing sublayer references. This path may be stale in documentation; check Git history rather than fabricating/restoring it.

## Other uncommitted content to preserve

- Modified: `config/rviz/echo_d455_front.rviz`, `config/rviz/echo_m04d_sensors.rviz`, `scenes/nav_baseline/navigation.usda`.
- New critical S03: `config/rviz/echo_d455_four_clouds.rviz`, `config/rviz/echo_d455_four_light.rviz`, `launch/echo_s03_four_rgbd_sync_probe.launch.py`, `scripts/start_echo_d455_four_live.sh`, rear/left/right TF and camera-info helpers, rear/side startup scripts, and seven new S01 derived layers.
- Untracked experiment data: `results/s03_four_mapping/*.db`, `*.ply`. **Do not stage large raw databases/PLY files as source code.** Preserve data locally and decide later on artifact retention and `.gitignore`.
- Only plausible *archive candidates* (NOT deletion candidates):
  - `config/agents/ira/warehouse_two_npcs_one_human.yaml.backup_20261109_234102`
  - `config/agents/ira/warehouse_two_npcs_one_human.yaml.backup_20261110_003152`
  - `scripts/start_full_warehouse_agents.py.backup_20261109_170108`
  - `tools/echo_s02_dual_sensor_trial.sh.bak_20261119_230321`
  Review these against the active files and search for references before relocating. Keep copies outside the repo; avoid `git clean`.
- Three untracked Nav2 office configurations are unrelated to this scene cleanup and should be preserved.

## Safe sequence

1. Keep `scenes/` intact. Treat the current path as the canonical entry point; don't create symlink aliases because USD relative sublayer resolution may depend on the file location.
2. Record an explicit `docs` scene entry-point index and working startup commands. This is the **safe immediate cleanup**: stop ambiguity between stage prototypes, static S03 and dynamic M04C.
3. Confirm real USD binary references using Isaac Sim's Python USD APIs. Verify local worker asset directory and effective runtime actor counts; the audit shows config counts, not a current runtime test.
4. Review only the four explicit `*.backup*`/`*.bak*` candidates; archive after confirming no active imports/references. Preserve all untracked edits.
5. Optional **future** consolidation: create a new derived/flattened four-camera scene *alongside* the old one using Isaac Sim; validate ROS topic parity, TF, physics, LiDAR and scene imagery before removing any old layer. Never overwrite original layers. A dynamic four-D455-plus-NPC scene is a separate integration task.
6. Create a scoped Git commit by specifying exact verified files, avoiding broad `git add .` or `git add -A`.

## Source limits

The archive includes inventory tables, snapshots of all 17 S01 USD-A layers, and selected launcher/configuration sources. It does **not** contain the five binary full-warehouse USD files or every repository script. Assertions about uninspected binary arcs, available asset references and runtime actor spawn counts remain unverified for this current run.
