# ECHO NAV-0 — verified odometry-frame navigation checkpoint

**Date:** 2026-11-23  
**Scope:** Minimal Nav2 bootstrap for autonomous motion in the S03 four-D455 Isaac Sim warehouse.  
**Relationship to `echo-nav-v1`:** This is a separate experimental integration. The proven 3D-LiDAR/AMCL Nav2 baseline remains unmodified.

## What was actually validated

- `odom -> base_link` and `base_link -> front_3d_lidar` TF resolved live.
- Robot USD active collider envelope (chassis coordinates): X −0.587 to +0.120 m; Y −0.238 to +0.238 m. The historical rectangular Nav2 footprint plus padding encloses it.
- S03 robot accepts `geometry_msgs/Twist` on `/cmd_vel`; `/chassis/odom` reports `odom` / `base_link`.
- The front LiDAR PointCloud2-to-LaserScan converter produced 362/362 finite, in-range readings, around 2.41 m nearest in the audited stationary scene. Scan is published in `front_3d_lidar` frame, covering front ±90°.
- Nav2 controller, planner, navigator, velocity smoother and Collision Monitor were all active. `/cmd_vel_smoothed`: smoother -> Collision Monitor. `/cmd_vel`: one publisher (Collision Monitor), one subscriber (Isaac Sim drivetrain).
- The first 0.30 m forward `odom`-frame Nav2 `NavigateToPose` goal reported `SUCCEEDED` from the goal client, and the navigation log reported `Reached the goal!` / `Goal succeeded`. The actual final displacement was not independently recorded.
- The DWB `FollowPath.xy_goal_tolerance` was fixed to **0.08 m**, while `general_goal_checker.xy_goal_tolerance` remains **0.10 m**. The older 0.25 m DWB tolerance caused premature rotation-only behavior.

**Not validated:** obstacle-triggered stopping, stale-LiDAR-source stopping with advancing simulation time, rear/side obstacle protection, unattended warehouse routing, accurate traveled-distance ground truth. Collision Monitor is not certified safety equipment.

## Launch from the ECHO repository

First terminal (normal ECHO supervisor):

```bash
cd ~/echo
ECHO_S03_DEPTH_CY_MODE=baseline python3 scripts/start_echo_dynamic_four_d455.py
```

Press Play in Isaac Sim. Do **not** run `scripts/start_echo_nav.sh` at the same time; that is the preserved, older map/AMCL baseline. Ensure no other NAV-0 or `pointcloud_to_laserscan` instance is active.

Second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch ~/echo/launch/echo_nav0.launch.py
```

The launch automatically reads `~/echo/config/nav2/nav0/echo_nav0_params.yaml` relative to its own file path. To override: `params_file:=/path/to/params.yaml`.

Third terminal, stationary preflight:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
ros2 lifecycle get /collision_monitor
ros2 topic info /cmd_vel --verbose
ros2 topic info /cmd_vel_smoothed --verbose
ros2 topic echo --once /scan --field header
```

Confirm all lifecycle nodes active, `collision_monitor` alone publishes `/cmd_vel`, and `/scan` contains **valid ranges** (not merely a topic name). For a supervised 0.30 m goal, with clear space ahead:

```bash
python3 ~/echo/scripts/echo_nav0_short_goal.py           # preview only
python3 ~/echo/scripts/echo_nav0_short_goal.py --execute # requires typing GO
```

The goal script retains a wall-clock watchdog; Ctrl+C requests Nav2 cancellation but is **not** a guaranteed emergency stop. If uncertain, stop the Isaac Sim simulation manually.

## Research use and limitations

NAV-0 runs short local goals with rolling `odom`-frame costmaps and **does not start AMCL, a map server, or RTAB-Map**. It provides neither full-warehouse planning nor a persistent occupancy grid. Previous four-camera RTAB-Map experiments used `publish_tf:=false`; a later map-based mode requires deliberate, unique ownership of `map -> odom` and a usable occupancy map.

**Keep mapping modalities separate:** 4×D455 RTAB-Map mapping should consume four synchronized RGB-D observations, while a later 3D-LiDAR-only baseline should consume the original PointCloud2 data. The front LiDAR scan used by NAV-0 for local obstacle avoidance is not itself a LiDAR mapping input. Compare on matched trajectories with independent map/GT metrics. Do not modify or replace `echo-nav-v1`.

## Remaining gate before extended mapping trajectories

Run one controlled Collision Monitor obstacle-stop and sensor-staleness test (while sim time advances). Until then, NAV-0 is limited to short, operator-supervised goals. Preserve the known-good checkpoint rather than undertaking a broad Nav2 refactor.
