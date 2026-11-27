# ECHO — Updated master roadmap
27 November 2026. Phase C closed; Phase D branch created. Replaces current-state portions of the 23 November roadmap. The A–I phase labels below retain the original roadmap (H tracking, I Nav2 integration); S04 subgates are proposed execution structure, not already agreed numerical acceptance thresholds.

| Phase | Scope | State |
|---|---|---|
| A / M04B | Actor GT logging and identity | Completed baseline; generic one-command logger deferred |
| B / M04C–D | Dynamic warehouse, worker RGB/LiDAR visibility | Completed at sensor scope; independent detection/tracking deferred |
| C / S01–S03 | Four-D455 integration, calibration, mapping | Closed at engineering scope; tag `echo-phase-c-v1`, merged `3ed6abe` |
| D / S04 | Controlled mapping and sensor comparison | Next; branch `echo/s04-sensor-comparison` |
| E / S05 | Global relocalization and revisits | Planned |
| F / M05 | Controlled environmental change scenarios | Planned |
| G / M07–M08 | Object perception and persistent memory | Planned |
| H / M06 | Dynamic tracking and motion prediction | Later |
| I / M09 | Nav2 integration and final evaluation | Later |

## Phase C disposition
Completed: four streams and aggregate, dynamic warehouse integration, supervised startup/shutdown, NAV-0, selected ideal geometry ground truth, same-input baseline/coupled XY comparison and rollback, main chassis pose vs ROS odometry consistency, short concurrent mapping pilot, database and stored calibration audit, native and offline map visualization, documentation and Git push. D8: 3/3 goals, 113 observations, nine retained poses, 452 corrected models, zero loop closures, 433,925 offline point samples. Not a complete mapping-quality or independent localization evaluation.

Transferred: temporal cross-view agreement (no useful simultaneous overlap in D6 sample), full route/turn coverage, matched sensor comparison, dynamic map contamination, explicit obstacle stop/stale-sensor tests. General Phase A actor logger convenience and advanced LiDAR tracking stay deferred; do not make them blockers unless needed for a specific experiment. Physical D455 and realistic wheel-odometry validation are separate from synthetic geometry.

## Phase D — bounded proposed execution plan
Goal: establish a fair geometric mapping comparison using the existing working sensors. S04 is not a new SLAM, object tracking or semantic perception project.

| Gate | Action | Exit evidence |
|---|---|---|
| S04-0 | Read-only inventory of sources, environment, stored runs, NAV-0 and existing S02 mapping/reconstruction scripts | Verified file paths, mapping inputs, TF ownership, recording and rollback plan; agree protocol |
| S04-1 | Supervised obstacle stopping and stale-sensor response in isolated simulation | Logs of commands, scan freshness, physical pose/clearance, stop response; no unsafe motion; abort and fix only if blocking |
| S04-2 | Static repeatable short route with straight segments, turns and a revisited surface; record GT and all sensors | Defined spawn/waypoints, speed, sim-time stamps, actual trajectory; usable clouds across multiple poses; no duplicate TF/velocity publisher |
| S04-3 | One-D455, four-D455 and 3D-LiDAR mapping using matched trajectory and common odometry | Three isolated mapping outputs and manifests, same evaluation region, verified input exclusivity; inspect native mapper and LiDAR baseline options before selecting method |
| S04-4 | Compare static coverage and temporal alignment; repeat with controlled moving agents vs stationary reference | Ground-truth or documented reference surface errors, coverage, ghosting evidence, timings and resource measurements; matched agent seeds and actual trajectories logged |
| S04-5 | Summarize results, limitations, choose reference map/configuration for S05, commit and tag | Reproducible protocol, tables/figures, archived raw data, clean Git checkpoint; no new gates unless reproducible blocker |

Prefer common recorded sensor replay for geometric comparison where feasible (capture synchronized streams, clock, TF, odom, scene GT once); otherwise use repeatable runs and quantify route mismatch. Separate all-sensor acquisition overhead from per-mapper processing. Use same external odometry for all three geometric pipelines, explicitly simulator-derived. If LiDAR and RGB-D map builders differ, report the comparison as sensor+pipeline rather than a pure sensor ablation. Compare native coverage and a common visible region separately. Do not equate total points with quality. Use consistent voxel size/range for evaluation where sensible, preserve native-range results separately. Measure bidirectional surface distances, completeness/coverage, temporal static-surface agreement, map size, update rates, CPU/GPU/RAM, and dynamic ghost occupancy using a documented reference. Use GT to score, not to give object identities to the eventual estimator. A practical starting proposal is one static and one moving-agent pilot, then three repeats per condition where feasible; agree duration, tolerances, repetitions and metrics before recording final data. If a repeat or metric is infeasible, label the missing evidence rather than invent a pass.

Safety: front-sector NAV-0 sensing is not omnidirectional obstacle protection. Avoid uncontrolled reverse or lateral/turn motion; inspect footprint and sweep clearance, use bounded supervised simulation tests and a clear cancellation/stop path. Never remove the collision monitor to force progress. Do not treat D8 success as safety certification.

## Later research (unchanged goals)
E: save controlled maps, known-pose sanity checks then unknown-start trials without GT pose initialization; compare one/four RGB-D and LiDAR localization, time to correct pose, success rate, position/yaw error under viewpoint change/occlusion and moving agents. F: controlled moved/appeared/removed/occluded object scenarios, identities, visibility and before/after/multi-visit GT. G: instance segmentation, RGB-D back-projection, cross-camera and revisit association, persistent identities/history/confidence; classify unchanged/moved/appeared/removed/unobserved with explicit visibility evidence; evaluate precision/recall and ID consistency. H: track people/robots, short-horizon constant velocity baseline, compare GT trajectory errors; distinguish moving actors from relocated objects. I: object-aware obstacle information, temporary vs persistent hazards, prediction-aware conflicts, compare standard Nav2 vs ECHO; quantify earlier red-NPC contact scenario and compute overhead, final report.
