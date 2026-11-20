# ECHO — Master Research and Engineering Plan

**ECHO:** Environmental Change and History of Objects  
**Plan version:** 2026-11-20 (initial versioned roadmap)  
**Code reference:** GitHub `main` at `5ccad850`; local `echo/s01-d455` at `5b40f2b` in the uploaded documentation audit.  
**Source of truth:** the actual `~/echo` working tree, verified ROS/Isaac measurements, and versioned experiment reports. This document states intentions and gates; it does not certify unperformed tests.

## 1. Research mission

ECHO investigates how a mobile robot can maintain a persistent, object-aware 3D representation of a changing environment. The robot must (i) construct a spatial map, (ii) recover its global pose on revisiting a previously mapped site, (iii) associate object identities between visits, and (iv) distinguish **unchanged, moved, appeared, removed, occluded, and insufficiently observed** objects. Dynamic-object tracking and navigation decisions based on persistent memory are optional subsequent research extensions.

The reference platform is a white, ROS-enabled NVIDIA Nova Carter in an NVIDIA Isaac Sim 6.2 warehouse, running ROS 2 Jazzy and Nav2. We will preserve the existing 3D LiDAR and navigation baselines while introducing four outward-facing **simulated D455-style** RGB-D sensors. These sensors are an interface approximation; simulated ideal depth is not an experimentally validated physical Intel RealSense D455 noise model.

This is an independent R&D project. Do not copy proprietary hardware CAD, private hospital data, credentials, or confidential employer material into the public repository.

### Research questions

- **RQ1 — Multi-view relocalization:** Does observing the environment through four calibrated RGB-D views increase unknown-start global relocalization success and reduce time-to-correct-pose versus a single RGB-D view, under matched test conditions?
- **RQ2 — Sensor configurations:** What are the strengths, limitations, runtime costs and map/localization characteristics of one simulated D455, four simulated D455s, and the preserved 3D LiDAR?
- **RQ3 — Revisit-aware change:** Given localization and incomplete observations, can an object-centric memory distinguish real appearance, removal and movement from occlusion or being out of view?
- **RQ4 (optional) — Dynamic/navigation utility:** What measurable benefit, if any, does persistent object state provide to moving-object handling or Nav2 obstacle updates?

**Research hypothesis vs established fact:** Multi-view RGB-D *may* improve coverage/relocalization. Four cameras do not guarantee it; greater compute cost, overlap, feature ambiguity and synchronization error could negate the gain. Sensor comparison is not a competition on point counts alone. RTAB-Map, AMCL and other established methods are baselines; novelty should be sought in repeatable multi-view evaluation and evidence-aware object memory, not claimed for existing SLAM algorithms.

## 2. State at this handover

| Stage | Current evidence and limitation | Status |
|---|---|---|
| M01/M02: warehouse, robot, Nav2 | Previously verified simulation navigation reference; preserve tagged baseline `echo-nav-v1`. | Historical baseline completed |
| M03: RGB-D/semantic prototyping | `raafif_office*.usd`, before/after captures and `results/experiment_01`–`05` exist on tracked `main`; integration with mobile ECHO is not established by the audit. | Early prototypes |
| M04C: dynamic actors | Two red NPC Carters, two animated workers, independently identified agents and 10 Hz ground-truth logging; see Phase B report. | Completed at defined scope |
| M04D: human crossing sensor study | Worker seen in RGB and 3D LiDAR, GT-gated nearby point statistics and temporal LiDAR change preview. No independent human detector validated. | Completed at defined scope |
| S01-A/B: front D455-like device | Working synthetic RGB/depth/CameraInfo/TF/point cloud; committed as `5b40f2b` on `echo/s01-d455`; see `docs/echo_s01_front_d455.md`. | Validated front interface |
| S02: front RGB-D mapping vs 3D LiDAR | RTAB-Map and dual-sensor trials reported in prior session; analysis/launch tools were *untracked* on 20 Nov. Raw datasets were not provided in audit ZIP. | Preliminary study, not re-audited here |
| S01-C: additional camera views | Rear/left/right are not yet installed and validated in the known working checkout. | **Next implementation** |
| S01-D: four-view RTAB-Map | One mapping process consuming all four synchronized RGB-D views, resulting in one map/database. | Proposed |
| R01/R02: evaluation | Controlled mapping and unknown-start relocalization. | Proposed |
| M05 onward: revisit changes | Change-event ground truth, object association, occlusion-aware memory, quantitative validation. | Proposed |

**Critical source distinction:** The uploaded ZIP provides tracked `main`/S01 branch trees and Markdown files, not the untracked S02 scripts, raw bags, live ROS topics, RTAB-Map database or USD graph contents. Before implementing or claiming reproducibility, inspect those locally.

## 3. Engineering principles and non-negotiable baselines

1. **Keep verified scenes and navigation stable.** Never overwrite `scenes/echo_warehouse_nav_reference.usda`, `scenes/full_warehouse/main_robot_ros.usd`, the Phase B test scenes or `scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda`. Create an isolated derivative for additional cameras.
2. **Keep the existing 3D LiDAR.** It is a baseline and independent observational modality, not a sensor to remove during four-camera development.
3. **Respect local work.** As of 20 Nov, local changes include RViz files, `scenes/nav_baseline/navigation.usda`, backup files, Nav2 configs and untracked S02 scripts. No `git reset --hard`, `git clean`, indiscriminate `git add -A`, or branch switching that risks these files.
4. **Use one robot and shared odometry in multi-camera mapping.** Four independent RTAB-Map maps are not an acceptable substitute for a single four-input mapping instance.
5. **Keep observation, inference and ground truth separate.** Do not use simulator object IDs/poses as inputs to the autonomous algorithm when they are meant to supply evaluation labels.
6. **Do not confuse a synthetic camera with hardware.** Reserve physical calibration, active-stereo interference, USB throughput/power, lighting and failure characterization for a separate sim-to-real phase.
7. **Small steps with exit gates.** Inspect actual USD layer stack, ROS graph nodes, existing scripts and local RTAB-Map version before changing code.

## 4. Work packages and exit criteria

### WP-C0 — Read-only front camera architecture audit (next)

**Inputs:** `scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda`, dependent S01 USD layers, `scripts/start_echo_d455_front_stack.sh`, `scripts/echo_d455_front_static_tf.py`, `scripts/echo_d455_front_depth_camera_info.py`, `docs/echo_s01_front_d455.md`.

**Actions:** Examine camera prims, mounting parents, stage units and references, Action Graph publishers/render products, ROS topic names, optical frames, TF ownership, CameraInfo/depth registration and helper startup/shutdown. Inspect the actual working scene, not just an assumed 3D model. Compare original Hawk mount coordinates only as provisional references.

**Gate:** A written table of actual prim paths, publishers, topics, frames, transforms, layer dependencies and choice of safe extension strategy. No scene edits during audit.

### WP-C1 — Rear camera alongside front

**Implementation:** Add a rear-facing simulated RGB-D assembly in an isolated derived scene or runtime extension. Reuse and parameterize the verified front strategy where safe. Retain front and 3D LiDAR, avoid duplicate frame publishers.

**Gate:** Front + rear simultaneously publish nonempty RGB, depth and CameraInfo, with unique frame IDs, valid optical TF, measured paired timestamps, visually correct rear view and appropriate depth geometry. Record per-stream rates, frames dropped, ROS/Isaac performance and any chassis occlusion. Do not proceed if rear images are merely a duplicate of front images.

### WP-C2 — Left/right and four-camera runtime

**Implementation:** Add lateral assemblies with measured/recorded mounting translations and orientations; fix registration/TF/clock issues as needed. Document the exact extrinsics and render configuration.

**Gate:** Four simultaneous RGB-D interfaces, each with verified orientation, unique topics/frames, registered depth or explicitly defined alignment, consistent sim time, independent image content and acceptable observed throughput. Preserve LiDAR availability and report compute cost.

### WP-D — Single RTAB-Map four-camera map

**Implementation:** Inspect installed `rtabmap_ros` multi-RGBD interface/version and configuration. Test one, two and four camera inputs into **one** RTAB-Map mapping process/database using shared `/chassis/odom`. Record which camera observations were accepted and how synchronization is handled.

**Gate:** One reproducible mapping run that demonstrably consumes all four views; nonempty database/map, coherent TF, repeatable launch procedure and logged input rates. This gate does **not** mean global relocalization already works.

### WP-R01 — Mapping sensor comparison

**Conditions:** One D455-like RGB-D; four D455-like RGB-D; one 3D LiDAR. Use identical scene state, robot trajectory/speed, reference frames, logging windows and independent simulator ground truth when possible. Keep native sensor properties visible in the report (range, FOV, rate) rather than artificially making them identical.

**Metrics:** Ground-truth referenced map completeness/occupancy or surface distances, overlap/coverage and blind spots, map consistency, latency, dropped observations, runtime FPS, CPU/GPU load and memory use. Point counts or D455↔LiDAR nearest-neighbor distance alone are not depth accuracy.

**Gate:** Repeatable trials with specified preprocessing, excluded/invalid observations and uncertainty, plus a balanced discussion of modality differences.

### WP-R02 — Global relocalization from unknown poses

**Implementation:** Build/save maps; start independent *localization* trials from predefined novel positions/yaws and orientations, without revealing test ground truth to the estimator. Use a reset/initialization protocol that does not silently provide correct `/map→odom` or prior trajectory; distinguish odometry propagation from absolute place recognition. RTAB-Map is the primary baseline. Evaluate LiDAR localization with an explicitly specified comparable backend rather than labeling AMCL versus RTAB-Map a pure sensor comparison.

**Metrics:** Fraction correctly localized under **predeclared** translation/yaw tolerances, time to first correct stable pose, false acceptances, pose errors on successes, no-solution counts, compute/performance. Report results for one and four D455 and a well-defined LiDAR baseline.

**Gate:** A dataset and script that independently check estimated global pose against simulator GT for a representative matrix of starts, scene appearances and partial occlusions.

### WP-M05 — Controlled environmental changes with ground truth

**Implementation:** Begin with one known box/pallet moved between visits while logging both 3D world poses and explicit change event labels. Add a newly appeared object, removed object, partially/fully occluded object, and unobserved/out-of-FOV cases. Separate changes between visits from people/robots moving during a visit.

**Gate:** Reproducible before/after world states, ground-truth event log, robot trajectories and sensor captures with no access to test GT in perception code.

### WP-M06 — Persistent semantic object memory

**Design candidate (not yet built):** RGB-D instance masks/detections → calibrated 3D observations → candidate association via geometric gating and appearance/semantic features → persistent instance ID, position/extent/class/uncertainty and observation history. Handle competing associations, disappearance and reappearance. Reuse lessons from the early office prototypes without treating them as mobile-robot validation.

**Gate:** Independent cross-visit object association with measured ID-switch rate, correct/incorrect matches and 3D pose errors under GT evaluation.

### WP-M07 — Occlusion-aware change reasoning

**Design candidate:** Visibility/ray reasoning plus evidence accumulation. Distinguish `REMOVED` from `NOT_OBSERVED`: a previously known object must have adequate coverage of its expected location and sustained negative evidence before removal is inferred. Track uncertainty and allow unresolved hypotheses rather than forcing a label.

**Gate:** Per-class precision/recall, confusion matrix, change-localization error, time to confirmation and explicit false-removal rate under occlusion, novel viewpoint and incomplete observation stress tests. Test ablations without visibility reasoning and without temporal memory.

### WP-M08 — Optional dynamic tracking and Nav2 integration

**Only after prior gates:** Compare static world memory against transient dynamic objects, explore short-horizon motion forecasts where supported by data, and optionally add an evidence-gated obstacle/update interface to Nav2. Keep safety-critical real-world navigation claims out of scope until validated.

**Gate:** Controlled tracking and navigation metrics over independent scenarios, including failure analyses. No promise that this extension will fit initial schedule.

### WP-E — Reproducibility and reporting

Record tested commit, scene layer chain, full launch commands, ROS/Isaac versions, calibration files, random seeds, GT method, simulator settings, exact recorded topics and configs, datasets/manifests, scripts, plots and limitations. Publish aggregate metrics and small figures; keep large raw MCAP/PLY/DB datasets outside Git unless an explicitly versioned data distribution is planned.

## 5. Evaluation design at a glance

- **Unit of comparison:** matched robot route/world state, not independently hand-driven paths; separately test different FOV coverage.
- **Training/mapping vs test:** map construction and localization trials have distinct runs; no map data from the test episode should leak into localization initialization.
- **Independent GT:** simulator poses and object event ground truth are evaluation inputs only.
- **Repetition:** choose a documented route/start matrix, repeated trials and fixed random seeds; predefine success tolerances before analyzing results.
- **Failure analysis:** lighting/texture, reflective or missing depth, near-field blind spots, dynamic occluders, insufficient features, synchronization loss, robot movement, false loop closures.
- **Reporting:** accuracy plus runtime/resource cost; include confidence intervals or variability, not only best runs. See `docs/ECHO_EXPERIMENT_PROTOCOL.md`.

## 6. Research outputs and claim discipline

| Output | Evidence required before claiming success |
|---|---|
| Four RGB-D sensors | Observed data on four distinct topics, verified extrinsics/optical frames and timing |
| One four-camera map | One RTAB-Map instance/database shown to ingest all four views |
| Global relocalization | Correct global pose from unknown starts, independently assessed against GT; success/time/error |
| Persistent objects | Cross-visit instance matching with measured ID continuity |
| Removed vs occluded | Explicit visibility evidence and measured false-removal rate |
| Navigation benefit | Ablation against Nav2 baseline, on comparable goals/routes and documented dynamics |

Earlier S02 sensor-cloud results are preliminary **spatial correspondences**, not independent sensor-accuracy calibration or a global localization result.

## 7. Risks and dependencies

| Risk | Mitigation or measurement |
|---|---|
| ROS TF/camera optical frame errors | Inspect parent relationships and rotation; validate transforms and projection with a known scene object |
| RGB/depth and multi-camera desynchronization | Measure timestamp matches, skew and dropouts over controlled windows; keep `/clock` and `use_sim_time` consistent |
| GPU/Isaac performance drop under four cameras | Compare 1/2/4 camera FPS and actual ROS message rates with/without LiDAR; adjust load transparently |
| Untracked S02 scripts and uncommitted local work | Review individually, commit only explicit relevant paths after validating them |
| External NVIDIA D455 USD dependency | Document asset URL/version/resolution, avoid assuming reproducibility without asset access |
| Relocalization GT leakage through odometry/map | Predefine clean initialization and independent GT-only scoring |
| Occlusion falsely labeled as removal | Visibility estimation and unresolved states, negative-evidence thresholds and occlusion-specific benchmarks |
| Sim-to-real gap | Treat physical D455 calibration, USB, IR/lighting and geometric limits as separate experiments |

## 8. Documentation and change control

- `README.md`: public entry point and high-level current status.
- `docs/ECHO_MASTER_PLAN.md`: this research roadmap and decision/exit gates.
- `docs/ECHO_ARCHITECTURE.md`: current vs planned interfaces, data flow, calibration and ownership.
- `docs/ECHO_EXPERIMENT_PROTOCOL.md`: controlled trials, metrics and reporting templates.
- `docs/echo_phase_b_m04c_m04d.md`: historical completed Phase B report — preserve unchanged.
- `docs/echo_s01_front_d455.md`: dated S01 checkpoint — preserve detailed evidence, retain its historical "RTAB-Map pending" wording as of 19 Nov, and link newer S02 report from README/master plan.
- `docs/echo_s02_preliminary_comparison.md`: subsequent reported front RGB-D/3D-LiDAR experiment with limitations.

**Workflow:** inspect → implement one small change → run validations → capture evidence → review diff → stage named files → commit → update relevant milestone documentation. Maintain one commit boundary per validated experiment when practical.

## 9. Immediate next action

The Git/file inventory is already complete; do not repeat it. **Read only** the S01 verified USD and dependency chain, launch stack, optical-TF helper and depth-CameraInfo helper, plus ROS camera publishers/graph prim paths. Only after auditing them, create an isolated front+rear scene. Leave navigation/LiDAR baseline scenes and uncommitted work untouched.
