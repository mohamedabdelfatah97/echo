# ECHO

**Revisit-aware 3D semantic object memory and environmental change detection for mobile robots**

ECHO explores how a robot can recognize and remember objects in a 3D environment, then identify what has **appeared, disappeared, moved, or remained unchanged** when it returns. The objective is persistent, object-centric scene understanding across repeated observations—not simply comparing two images pixel by pixel.

> **Status:** A single Nova Carter robot can autonomously navigate NVIDIA's warehouse in Isaac Sim 6.2 using ROS 2 Jazzy and Nav2. Initial object-memory experiments were completed with a fixed RGB-D camera. Integrating the perception pipeline with the moving robot is the next milestone.

## Research objectives

- Maintain persistent identities and 3D positions for observed objects across visits.
- Detect newly introduced, removed, and relocated objects.
- Distinguish genuine environmental change from viewpoint differences and temporary occlusions.
- Use robot poses and RGB-D observations to associate instances across multiple viewpoints.
- Evaluate change detection with controlled, repeatable simulations and quantitative metrics.

## Technology stack

| Component | Technology |
| --- | --- |
| Simulator | NVIDIA Isaac Sim 6.2 |
| Robot and environment | Nova Carter in NVIDIA's warehouse |
| Robot navigation | ROS 2 Jazzy, Nav2, AMCL, LiDAR, TF and odometry |
| Computer vision | Python, OpenCV, NumPy |
| Segmentation | SAM 2.1 (PyTorch/CUDA) |
| Development OS | Ubuntu 24.04, NVIDIA |

## Verified navigation baseline

The local ECHO warehouse scene has been tested with **one Nova Carter** and **one Nav2 stack**. The robot received an RViz navigation goal, moved in Isaac Sim, and successfully reached its destination.

The following have been verified:

- Simulation clock (`/clock`) and robot odometry (`/chassis/odom`).
- Raw 3D LiDAR point clouds and converted 2D laser scans (`/scan`).
- AMCL localization and the `map → odom → base_link` transform chain.
- Active Nav2 controller, planner, AMCL, and behavior-tree navigator.
- Live robot RGB camera output.
- Autonomous goal execution in the saved **meter-scale** warehouse scene.

**Not yet verified:** A robot-mounted RGB-D/depth stream suitable for ECHO's 3D reconstruction pipeline. The RGB image alone does not establish working depth perception.

### Start the simulation

These launchers assume Isaac Sim is installed at `~/isaacsim`, the NVIDIA ROS workspace is installed at `~/IsaacSim-ros_workspaces/jazzy_ws`, and ROS 2 Jazzy is available under `/opt/ros/jazzy`.

**Terminal 1 — Isaac Sim, local warehouse scene, automatic playback**

```bash
~/echo/scripts/start_echo_sim.sh
```

**Terminal 2 — Nav2 and saved RViz layout**

Start this after the simulator is publishing `/clock`:

```bash
~/echo/scripts/start_echo_nav.sh
```

Select **Nav2 Goal** in RViz and send the robot to a clear location on the warehouse map.

### Navigation files

| File | Purpose |
| --- | --- |
| `scenes/echo_warehouse_nav_meters.usda` | Meter-scale local USD layer referencing NVIDIA's warehouse simulation |
| `scripts/start_echo_sim.sh` | Opens Isaac Sim and starts simulation playback automatically |
| `scripts/start_echo_nav.sh` | Starts the single-Carter Nav2 stack and RViz |
| `launch/echo_carter_navigation.launch.xml` | ECHO copy of NVIDIA's Carter navigation launch with the saved RViz config |
| `config/rviz/echo_carter_navigation.rviz` | Saved RViz visualization and panel layout |

The working NVIDIA navigation packages, map, and parameter files remain in the separate ROS workspace. The USD scene refers to external NVIDIA assets; it is **not** a standalone asset bundle. The custom launch currently uses an absolute path for the RViz configuration, which must be updated if the repository is moved to a different home directory.

## Perception experiments completed

The first five experiments used a **controlled fixed-camera RGB-D setup**, separate from the moving-robot navigation baseline.

| Experiment | Purpose | Outcome |
| --- | --- | --- |
| 01 | RGB-D image differencing | Detected regions changed between two observations |
| 02 | Prompted instance segmentation | Segmented test boxes with SAM 2.1 |
| 03 | Depth-based 3D displacement | Reconstructed box positions and estimated movement |
| 04 | Persistent object association | Matched before/after instances using 3D center distances |
| 05 | Automatic SAM 2 mask proposals | Demonstrated automatic proposals and their limitations on cluttered boxes |

**Controlled result:** In one fixed-camera experiment, an object was moved by 15 cm. ECHO estimated its displacement with approximately **3.84 mm displacement-vector error**. This is a **single-scenario result**, not a claim of general system accuracy or performance under mobile viewpoints.

The original prototype was named *RAAFIF*. Some historical experiment scripts and files retain that name.

## Planned ECHO pipeline

```text
Robot RGB-D observations + robot poses
                 |
                 v
      Object detection / segmentation
                 |
                 v
       3D object localization
                 |
                 v
      Persistent instance association
                 |
                 v
       Revisit-aware change reasoning
                 |
                 v
    Updated 3D object memory + change log
```

The next development steps are to verify robot-mounted RGB and depth data, connect moving-camera observations to 3D object memory, and evaluate controlled object relocation, removal, appearance and occlusion across autonomous revisits. Moving people and other robots are future dynamic-agent scenarios, **not yet implemented ECHO capabilities**.

## Repository layout

```text
echo/
├── scenes/          # Isaac Sim scenes and USD layers
├── scripts/         # Simulation launchers and perception experiments
├── launch/          # ECHO-specific ROS launch configuration
├── config/rviz/     # Saved RViz configuration
├── data/            # Experimental RGB-D inputs and metadata
├── results/         # Experiment outputs and visualizations
└── third_party/     # External research code/checkpoints (not distributed here)
```

Model weights, Python virtual environments, caches, and large third-party assets should not be committed. Model and asset licenses remain with their respective authors. Experiment data may be local-only, depending on size and provenance.

## Evaluation roadmap

ECHO will be evaluated on object association across visits; moved/new/removed-object detection; 3D localization error; sensitivity to viewpoint change and partial occlusion; and repeatability under controlled simulation conditions.

## Notes

This is an evolving research prototype, not a production robotics system. Navigation and fixed-camera perception have been tested **separately**; the combined moving-robot perception and environmental-change detection pipeline is the next research milestone.
