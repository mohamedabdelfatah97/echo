# ECHO — Experimental Protocol, Metrics and Reproducibility

**Version:** 2026-11-20, prospective research protocol (not a record of executed future experiments).  
**Scope:** Sensor mapping/relocalization and revisit-aware environmental change assessment, with optional later dynamic/Nav2 experiments.

## 1. Core principles

1. Define trial conditions and success thresholds **before** looking at test outcomes.
2. Distinguish simulator ground truth (GT), robot odometry, map/localization estimates, perceived instance detections, and manually labeled observations.
3. Do not expose GT to the estimator when evaluating autonomous localization/object memory. GT may define scene setup and off-line labels.
4. Use reproducible world/scenario configurations and fixed random seeds, report repeated-run variability and failure cases.
5. Avoid conflating sensor differences with different SLAM algorithms, maps, resolutions or observation rates. Explicitly document unavoidable cross-modal backend differences.
6. Never interpret nearest-neighbor point cloud correspondence as unbiased range/depth accuracy without a visible surface/GT measurement model.
7. Document what was actually measured and which quality gates failed; do not silently drop failed trials.

## 2. Baseline configurations

| ID | Sensors for target algorithm | Purpose | Status |
|---|---|---|---|
| C1 | One front simulated D455-like RGB-D | Existing RGB-D mapping reference | Front pipeline and preliminary S02 map observed |
| C4 | Front/rear/left/right D455-like RGB-D | Multi-view mapping and relocalization | Pending implementation |
| L3 | Existing 3D LiDAR | Independent map/localization and geometry baseline | LiDAR published; dedicated controlled localization comparison pending |

**Constants to strive for:** Same warehouse geometry; object placement; robot start/trajectory/speed; simulated time window; odometry source and GT capture; logged evaluation region; comparable map resolution/aggregation policy. **Report, don't conceal:** unique FOV, range, sample rate, modality noise, calibration and software backend.

**Ablations/secondary conditions:** C1 versus C1+rear, four sensors with one view withheld, static versus dynamic environment, reduced texture/occlusion, and one-versus-four with matching total compute budget where meaningful.

## 3. Experiment M — Mapping coverage and geometry

### Procedure

1. Freeze scene version and commit, sensor extrinsics, calibration, ROS launch and simulator settings.
2. Specify a deterministic route or odometry command playback; use the same route for each condition and verify with simulator GT.
3. Capture calibrated observations, `/clock`, TF, odometry, GT and simulator performance. Record both valid and invalid depth pixels.
4. Construct the map with the documented mapping backend; export the representations and metadata used in scoring.
5. Compare against independent ground-truth scene geometry or an explicitly labeled approximated reference. Define evaluation volume/height/visibility masks to exclude inaccessible areas consistently.
6. Repeat on multiple routes and with occluding dynamic agents as separate conditions.

### Suggested metrics

- **Coverage/completeness:** visible traversable/surface area covered, observed voxels relative to a GT/eligible region, blind-spot size, near-field coverage and FOV overlap.
- **Geometric consistency:** source-to-GT surface/occupancy distance and/or symmetric distances over explicitly co-visible surfaces, with p50/p90/p95 and thresholds.
- **Temporal/performance:** actual sensor Hz, timestamp skew, dropped frames, elapsed mapping time, simulator FPS, CPU/GPU load, RAM/VRAM, output map size.
- **Mapping trajectory:** GT-referenced ATE/RPE if the tested algorithm actually estimates pose; when RTAB-Map consumes external `/chassis/odom`, label that source and avoid crediting its accuracy to visual SLAM.

**Previous S02 note:** The 19 Nov one-D455/LiDAR nearest-neighbor overlay is a preliminary spatial comparison, not a substitute for this protocol.

## 4. Experiment R — Global relocalization from unknown starts

### Preparation

- Build and freeze a map using the declared mapping condition and a separate mapping session. Save map/database and calibration with checksums or version tags.
- Select a predeclared matrix of initial robot positions/yaws: familiar locations, altered headings, weak-texture zones, occlusions, and newly moved objects. Reserve some starts not near the recorded mapping trajectory.
- Reset or explicitly record prior localization state so that success cannot be trivially inherited from a correct supplied `map→odom`, initial-pose hint, or complete GT trajectory.
- Ensure startup odometry reflects only local relative motion. Record when and how each system receives its initial global pose (if any), and report separately protocols with/without hints.

### Trial and success criteria

1. Reset the localization backend and record trial ID, map ID and true GT initial pose in a **GT-only evaluation file**.
2. Start at the unknown pose; optionally rotate/drive according to a predefined observation policy. Log all attempted place recognitions, accepted poses, TF, estimated uncertainty, and wall-clock/sim-time timestamps.
3. Declare a **correct relocalization** only when global translation and heading errors are below *pre-registered* thresholds, and the solution remains consistent for a specified dwell period. Set these numerical tolerances before testing; do not select them after inspecting errors.
4. Count false positives (confident wrong global poses), timeout/no solution, all restarts and recovery times. Use independent simulator GT for scoring.
5. Run a matched number of repeated trials per configuration and report confidence intervals/uncertainty.

### Metrics

- Success probability with denominator **all attempted trials**, not only returned poses.
- Time to *first correct stable global pose*; censored failures at the trial timeout.
- Translation (m) and yaw (degrees) errors, both at first acceptance and after stabilization.
- False relocalization/false loop closure rate; timeouts and number of distinct places matched.
- Processor/GPU load and effective observation rate.

**Fairness caveat:** AMCL or a LiDAR-based method may use a different map/observation model from RTAB-Map. Report algorithm-conditional performance; don't claim a pure hardware win if software differs materially.

## 5. Experiment C — Revisit-aware object changes

### Controlled cases

| Case | GT change | Required evaluation question |
|---|---|---|
| C0 | No change | How many false change alarms? |
| C1 | Object relocated | Is the same object ID associated with the new pose; how accurate is motion estimate? |
| C2 | Object newly appeared | Is the new instance detected without duplicate ID assignment? |
| C3 | Object removed | Is removal concluded only after sufficiently observing the previously occupied region? |
| C4 | Object occluded / outside all FOV | Does the system correctly abstain from removal? |
| C5 | Multiple similar objects swapped or partly observed | Does association preserve identity or acknowledge uncertainty? |
| C6 | Moving agents pass through scene during revisit | Does transient occupancy contaminate persistent-object memory? |

### Procedure

1. Create separate `before` and `after` world scenarios with explicit simulator object IDs and GT poses in the **evaluation-only** logger; annotate occluders/visibility windows.
2. Follow fixed robot paths, then deliberately vary viewpoint and partial-visibility conditions.
3. Run detector/segmenter, depth-to-3D reconstruction, object association and memory **without access to GT object IDs or before/after labels**.
4. Require explicit observation evidence for `REMOVED` versus `NOT_OBSERVED/OCCLUDED/UNRESOLVED`.
5. Compare estimated instance events to GT; maintain per-event temporal history and uncertainty.

### Metrics

- Per-class precision, recall, F1 and a complete confusion matrix (`UNCHANGED`, `MOVED`, `APPEARED`, `REMOVED`, `OCCLUDED/UNRESOLVED`).
- Association match precision/recall, ID-switch counts, track fragmentation and wrong identity merges.
- 3D object pose/centroid error, displacement-vector error and geometric size error where applicable.
- False-removal rate in occlusion/unobserved cases; decision latency and fraction of justified abstentions.
- End-to-end throughput and memory storage overhead.

### Useful ablations

No temporal memory, no visibility model, no appearance features, no multi-view evidence, perfect GT localization versus estimated localization, and synthetic/noisy depth conditions. Identify which change failures are caused by localization vs object association vs visibility reasoning.

## 6. Experiment D — Dynamic object / navigation extension (optional)

Only if object-change milestones succeed: moving-agent trajectory estimation under GT, short-horizon prediction errors at fixed horizons, and Nav2 goal success/time/collision and obstacle-clearance latency. Evaluate with and without object-memory information under matched episodes; do not claim improved safety from qualitative visualization alone.

## 7. Data layout and trial manifest template

Each run should have a manifest stored next to its local outputs and a small reviewable summary under version control:

```yaml
experiment_id: S01C_rear_001
experiment_type: sensor_integration
status: proposed              # proposed | running | completed | invalid
repo_commit: null            # exact hash when performed
scene_path: null              # inspected USD path/layer chain
isaac_version: '6.2'
ros_distro: jazzy
sensor_configuration: [front, rear]
sensor_calibration_paths: []
ros_launch_paths: []
map_id: null
scenario_seed: null
sim_start_time: null
sim_duration_s: null
odometry_source: /chassis/odom
gt_source: null
ros_topics_recorded: []
expected_qa: [rgb, depth, camera_info, tf, timestamps, rates]
actual_metrics: {}
failures_and_exclusions: []
raw_data_local_dir: null      # large artifacts remain outside Git
summary_figure_paths: []
notes: null
```

**Naming:** Use unambiguous run IDs with dates, sensor condition and seeds. Store raw bags/DBs/PLYS locally in `diagnostics/` (and ensure `.gitignore` behavior is actually verified). Keep lightweight versioned figures, aggregated CSV/JSON and reproducible analysis commands in the repository.

## 8. Evidence ladder / publication checklist

| Claim | Minimum evidence |
|---|---|
| Sensor installed | Prim hierarchy and correct physical orientation only — **not** proof of output |
| Working RGB-D interface | ROS Image, depth, CameraInfo, TF, timestamps and repeatable observations |
| Multi-camera ingest | One mapping node demonstrably consumes all views; nonempty shared DB/map |
| Better mapping | Matched runs and GT-referenced metric differences with variability |
| Better relocalization | Independent unknown-start trials and GT pose check, all-attempt success/time/error |
| Object change detected | GT event classification with uncertainty and occlusion controls |
| Navigation benefit | Controlled ablation on goal/collision metrics with known limitations |

A figure, a single success demo, or a simulator GT-gated point search is useful evidence for its **stated scope**, not stronger claims.

## 9. Short-term experiment queue

1. **S01-C0:** Read-only inspection of verified front S01 scene, graph and helpers.
2. **S01-C1:** Two-camera front+rear data/TF/timing/performance report.
3. **S01-C2:** Four-camera independent stream validation.
4. **S01-D:** One four-view RTAB-Map ingest/map check.
5. **R01/R02:** Controlled sensor map and unknown-start relocalization comparisons.
6. **M05+:** Object-change event GT, association, occlusion-aware inference and ablations.

No new performance measurements are asserted by this document.
