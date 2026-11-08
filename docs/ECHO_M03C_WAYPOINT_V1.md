# ECHO M03C — repeatable waypoint pilot (visual-only)

## Why this version

The v1.3 probe established that `SimulationManager.get_simulation_time()` advances and that
an async frame loop can apply USD transforms. M03C generalizes that experiment to
configuration-driven waypoint sequences without touching the saved Nav2 baseline.

**Important:** The red robot uses `No_Physics`. It has no physical collision constraints,
obstacle avoidance, or guaranteed LiDAR visibility. The route is NOT automatically safe.

## Install

From Ubuntu: `cd ~/echo && unzip -n ~/Downloads/echo_m03c_waypoint_v1_patch.zip -d ~/echo`

Launch the existing experimental scene:
`~/echo/scripts/start_echo_red_patrol_v1.sh`

Do not launch another Isaac Sim instance at the same time.

## Dry run (default)

In **Isaac Sim → Window → Script Editor**, run:

```python
from pathlib import Path
exec((Path.home() / "echo/tools/echo_red_waypoint_v1_in_isaac.py").read_text())
```

From Ubuntu, read:
`cat ~/echo/results/red_waypoint_v1/red_waypoint_v1_report.json`

The starter route is the existing 20 cm +Y and back (x=2,y=-3 to x=2,y=-2.8),
using `heading_mode=hold_initial`. It has worked as an isolated translation;
**this does not prove any other route or in-place turn is unobstructed**.
Visually confirm the full swept body footprint, not just the waypoint centers.
Keep the main robot away from this area while testing.

## Arm after clearance verification

Edit `~/echo/config/agents/echo_red_waypoints_v1.json` and change:
`"armed": false` to `"armed": true`.

Rerun the same script in Isaac Sim. Wait for complete status, then inspect:

```bash
cat ~/echo/results/red_waypoint_v1/red_waypoint_v1_report.json
ls -lh ~/echo/results/red_waypoint_v1/red_waypoint_v1_*.csv
```

Return `"armed"` to `false` before committing the config.

## Add longer routes later

Use the same JSON file and add world-coordinate waypoints in its `waypoints` array,
only after checking clearance against shelves, forklift and other actors.
`heading_mode=face_motion` will rotate toward each next waypoint, but turning
expands the swept footprint; verify it independently before enabling.

Restrictions: <=8 destinations, <=1 m per leg (or stricter configured max),
<=4 m total (or stricter configured max), <=120 simulation seconds.
The pilot requires ending at its starting XY position by default.

## Stop

In Script Editor:

```python
from pathlib import Path
exec((Path.home() / "echo/tools/stop_echo_red_waypoint_v1_in_isaac.py").read_text())
```

Cancel restores the starting pose if the same stage remains open. Do not save the
stage during movement. Stop/close Isaac Sim only after the controller stops.

## What passes M03C

- Actor reaches each configured waypoint and returns to start
- Report status `complete`; physics time advances
- CSV shows timestamped XYZ/yaw samples; no pose jumps
- Main robot's topic publishers are unchanged
- Separate LiDAR/camera detection tests still pending
