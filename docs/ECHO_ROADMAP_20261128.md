# ECHO — Master roadmap (28 November 2026)

Replaces current-status portions of the 27 November roadmap. Phase D experimental scope closed by operator agreement, archive/Git release pending verification. Earlier A–I naming retained.

| Phase | Objective | State |
|---|---|---|
| A / M04B | Actor GT logging and identity | Completed baseline |
| B / M04C–D | Dynamic warehouse, worker RGB/LiDAR visibility | Completed sensor scope |
| C / S01–S03 | Four-D455 integration and calibration | Closed; `echo-phase-c-v1` |
| D / S04 | Controlled mapping and sensor comparison | Engineering-pilot experiments complete; S04-5 archive and release |
| E / S05 | Global relocalization and revisits | Next |
| F / M05 | Controlled environmental change scenarios | Planned |
| G / M07–M08 | Object perception and persistent memory | Planned; candidate original method |
| H / M06 | Dynamic tracking and motion prediction | Later extension |
| I / M09 | Nav2 integration and final evaluation | Later extension |

## Phase D disposition

See `ECHO_PHASE_D_CLOSEOUT_20261128.md` for evidence. S04-0 inventory; S04-1 bounded obstacle/stale monitor qualification; S04-2 static short route and extended completed prefix; S04-3 matched direct projection and static native RTAB-Map comparison; S04-4 preliminary moving-worker contamination vs actor-free map; S04-5 archive, report, reproducibility tools, checkpoint. The full planned route, independent absolute geometry accuracy, stationary-vs-moving matched actors, nearby NPC contamination, repeated statistics, full temporal cross-camera correspondence, dynamic RTAB optimization and CPU/GPU/RAM profiling are deferred, not passed. These do not block S05 pilot. No calibration reopening or additional S04 gate unless a reproducible blocker appears.

## Phase E proposed gates (agree exact thresholds before testing)

| Gate | Objective | Evidence |
|---|---|---|
| S05-0 | Inspect installed RTAB-Map interfaces, prior DBs and map/odom frame ownership | Reproducible read-only audit, reference selection, no duplicate TF |
| S05-1 | Acquire sufficient static reference-map coverage; preserve native DB | Reference map coverage and stored calibration, map pose/GT alignment, frozen copy/hash |
| S05-2 | Known-pose sanity check and localization-only pipeline | No reference DB growth, correct map-frame estimate and timestamped GT |
| S05-3 | Independent unknown-start revisits | Success/time/false acceptance/XY-yaw errors, opposite viewpoint, partial view/occlusion, explicit failure policy |
| S05-4 | Compare one/four cameras and LiDAR on matched query conditions | Same pose sets, no GT pose leakage; report pipeline distinctions and rates |
| S05-5 | Close S05 with reference and repeatable revisit protocol | Evidence, report, tag, handoff F |

These are proposals, not already agreed numerical criteria or completed milestones. Use GT only for scoring. Preserve independent pose resets, no continuous odom from mapping run in unknown-start query; avoid conflating local proximity constraints with global relocalization. Four-camera 3D-to-3D visual registration is candidate baseline; unsupported PnP with missing OpenGV found and bypassed for current mapping. Do not rebuild unless required by an agreed test. Use one-terminal supervised workflow.

## F and G — core research

F: controlled unchanged/moved/appeared/removed/occluded cases, identical-instance swaps, change magnitude, known GT identity/pose and visibility, reproducible multi-visit observations. G: existing segmentation and feature models, RGB-D back-projection, cross-camera/revisit association and persistent identity. Proposed original joint decision rule combines multi-view appearance and geometry, localization uncertainty, visibility/negative evidence, and accumulated competing hypotheses over visits. Baselines: category + nearest 3D, appearance + position, staged ECHO ablations, at least one external method if compatible. Evaluate change-class F1, ID preservation, false removal, pose errors, confidence/abstention, decision latency. Existing literature already addresses many components: Khronos, OASIS-Map, LTC-Mapping, ROMAN, ObVi-SLAM, SceneDiff. These are comparison leads from the literature discussion, not executed competitors; verify exact methods and current availability before claiming novelty. 3RScan is a candidate external dataset; no physical validation yet. Distinguish design from proven contribution.

H: dynamic tracking and short-horizon prediction against GT, constant-velocity baseline, distinguish transient moving agents from relocated static objects. I: history-aware temporary vs persistent obstacle information, prediction-aware conflicts, compare standard Nav2 vs ECHO and quantify overhead. Do not use these extensions as prerequisites for evaluating G. Keep original Phase A–C references and prior roadmap history.
