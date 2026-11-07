"""ECHO M03B — one bounded red-Carter translation using PHYSICS simulation time.

Run in ISAAC SIM SCRIPT EDITOR ONLY, with the experimental scene loaded.
Does not save the scene, control Nav2, or add ROS publishers. The actor's
No_Physics variant means this is visual/visibility validation ONLY.

BEFORE ARMING: visually confirm the ENTIRE swept robot footprint, not just
its centerline, is clear for DISTANCE_M along (DIR_X, DIR_Y), and that no
active robot is nearby. Default ARMED=False to prevent accidental motion.
"""
import asyncio
import builtins
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Usd

# ---- TEST SETTINGS: edit here before running ----
ARMED = False  # Set True ONLY after checking the whole path is unobstructed.
DIR_X = 0.0    # World +Y direction; adjust if another direction is clearer.
DIR_Y = 1.0
DISTANCE_M = 0.20
TRAVEL_TIME_S = 4.0        # each leg, simulation seconds; peak speed 0.075m/s
PAUSE_AT_END_S = 1.0      # simulation seconds
# -----------------------------------------------

ROOT = Path.home() / "echo"
SCENE_NAME = "echo_warehouse_dynamic_red_patrol_v1.usda"
PRIM_PATH = "/World/ECHO_Dynamics/Robots/Red_Carter"
RESULTS = ROOT / "results" / "red_patrol_v1"
TASK_KEY = "_echo_red_translate_v13"
REPORT = RESULTS / "red_translate_v13_report.json"


def smoothstep(u):
    u = min(1.0, max(0.0, float(u)))
    return u * u * (3.0 - 2.0 * u)


def write_report(payload):
    RESULTS.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2), encoding="utf-8")


async def run_test():
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage open")
    layer_id = stage.GetRootLayer().identifier
    scene_name = Path(layer_id).name
    if scene_name != SCENE_NAME:
        raise RuntimeError(f"Expected {SCENE_NAME}; currently {scene_name}")
    prim = stage.GetPrimAtPath(PRIM_PATH)
    if not prim.IsValid():
        raise RuntimeError(f"Red Carter missing: {PRIM_PATH}")
    pos_attr = prim.GetAttribute("xformOp:translate")
    if not pos_attr.IsValid():
        raise RuntimeError("Missing xformOp:translate")
    p = pos_attr.Get()
    initial = (float(p[0]), float(p[1]), float(p[2]))
    magnitude = math.hypot(DIR_X, DIR_Y)
    if not magnitude or not all(math.isfinite(x) for x in (DIR_X, DIR_Y, DISTANCE_M, TRAVEL_TIME_S, PAUSE_AT_END_S)):
        raise ValueError("Invalid direction or non-finite setting")
    if not (0 < DISTANCE_M <= 0.25 and TRAVEL_TIME_S >= 3 and PAUSE_AT_END_S >= 0):
        raise ValueError("Pilot limited to <=0.25 m, >=3 s per leg, nonnegative pause")
    dx = DISTANCE_M * DIR_X / magnitude
    dy = DISTANCE_M * DIR_Y / magnitude
    endpoint = (initial[0] + dx, initial[1] + dy, initial[2])

    base = {
        "scene": layer_id,
        "actor": PRIM_PATH,
        "start_xyz_m": initial,
        "end_xyz_m": endpoint,
        "travel_distance_m": DISTANCE_M,
        "travel_sim_time_s": TRAVEL_TIME_S,
        "source": "Isaac Sim SimulationManager.get_simulation_time()",
        "interpretation": "Authored visual USD pose; NOT physics-constrained motion or validated perception",
        "warning": "No collision avoidance / no rigid-body collision guarantees; never save during movement"
    }
    if not ARMED:
        write_report({**base, "status": "NOT_ARMED", "message": "Inspect full swept footprint then set ARMED=True at top of script"})
        print("ECHO M03B not armed. Proposed position", initial, "->", endpoint)
        print("Check full robot clearance; then edit ARMED=True and run again.")
        print("Report:", REPORT)
        return

    if not omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Press Play in Isaac Sim, then run the motion test")

    RESULTS.mkdir(parents=True, exist_ok=True)
    logfile = RESULTS / f"red_translate_v13_{datetime.now():%Y%m%d_%H%M%S}.csv"
    started_wall = time.monotonic()
    observed_sim_start = None
    last_sim_time = None
    updates = 0
    moved_max_m = 0.0
    status = "running"
    report_extra = {"log_csv": str(logfile)}
    try:
        with logfile.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["update", "physics_sim_time_s", "elapsed_sim_s", "x_m", "y_m", "z_m", "distance_from_start_m"])
            # Wait until PHYSICS time advances, without moving the actor.
            previous = None
            for _ in range(150):
                await omni.kit.app.get_app().next_update_async()
                now_stage = omni.usd.get_context().get_stage()
                if now_stage is None or now_stage.GetRootLayer().identifier != layer_id:
                    raise RuntimeError("USD stage changed during preflight")
                reading = float(SimulationManager.get_simulation_time())
                if previous is not None and reading > previous + 1e-5:
                    observed_sim_start = reading
                    break
                previous = reading
            if observed_sim_start is None:
                raise RuntimeError("Physics simulation time did not advance during 150 app updates. No movement attempted")
            last_sim_time = observed_sim_start
            print("ECHO M03B: Physics time advancing; translation armed. Start:", initial, "Target:", endpoint)
            total_time = 2 * TRAVEL_TIME_S + PAUSE_AT_END_S
            last_log_elapsed = -1.0
            for i in range(2500):
                await omni.kit.app.get_app().next_update_async()
                now_stage = omni.usd.get_context().get_stage()
                if now_stage is None or now_stage.GetRootLayer().identifier != layer_id:
                    raise RuntimeError("USD stage changed; motion aborted")
                if not omni.timeline.get_timeline_interface().is_playing():
                    raise RuntimeError("Simulation paused; motion aborted")
                sim_t = float(SimulationManager.get_simulation_time())
                if sim_t + 0.001 < last_sim_time:
                    raise RuntimeError("Physics simulation time jumped backwards; motion aborted")
                last_sim_time = sim_t
                elapsed = max(0.0, sim_t - observed_sim_start)
                if elapsed < TRAVEL_TIME_S:
                    fraction = smoothstep(elapsed / TRAVEL_TIME_S)
                elif elapsed < TRAVEL_TIME_S + PAUSE_AT_END_S:
                    fraction = 1.0
                elif elapsed < total_time:
                    fraction = 1.0 - smoothstep((elapsed - TRAVEL_TIME_S - PAUSE_AT_END_S) / TRAVEL_TIME_S)
                else:
                    fraction = 0.0

                commanded = (initial[0] + dx * fraction, initial[1] + dy * fraction, initial[2])
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    pos_attr.Set(Gf.Vec3d(*commanded))
                readback = pos_attr.Get()
                actual_delta = math.hypot(float(readback[0]) - initial[0], float(readback[1]) - initial[1])
                moved_max_m = max(moved_max_m, actual_delta)
                updates += 1
                if elapsed - last_log_elapsed >= 0.1 or elapsed >= total_time:
                    writer.writerow([updates, f"{sim_t:.6f}", f"{elapsed:.6f}",
                                     f"{readback[0]:.6f}", f"{readback[1]:.6f}",
                                     f"{readback[2]:.6f}", f"{actual_delta:.6f}"])
                    fh.flush()
                    last_log_elapsed = elapsed
                if elapsed >= total_time:
                    status = "complete"
                    break
            else:
                status = "error: 2500 app updates reached before simulation-time sequence completed"
    except asyncio.CancelledError:
        status = "cancelled"
        raise
    except Exception as exc:
        status = f"error: {exc!r}"
        print("ECHO M03B:", status)
    finally:
        # Restore actor only if the same scene is still open. Session-only edits.
        restored = False
        try:
            active = omni.usd.get_context().get_stage()
            if active is not None and active.GetRootLayer().identifier == layer_id:
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    pos_attr.Set(Gf.Vec3d(*initial))
                readback = pos_attr.Get()
                restored = all(abs(float(readback[j]) - initial[j]) < 1e-6 for j in range(3))
        except Exception as exc:
            status += f"; restore_error: {exc!r}"
        report = {**base, **report_extra,
                  "status": status,
                  "updates": updates,
                  "physics_time_start_s": observed_sim_start,
                  "physics_time_end_s": last_sim_time,
                  "physics_time_span_s": (last_sim_time - observed_sim_start) if (last_sim_time is not None and observed_sim_start is not None) else None,
                  "max_readback_translation_m": round(moved_max_m, 6),
                  "restored_initial_pose": restored,
                  "wall_elapsed_s": round(time.monotonic() - started_wall, 3)}
        write_report(report)
        print("ECHO M03B finished:", status, "max move:", round(moved_max_m, 3), "m; restored:", restored)
        print("Report:", REPORT)


# Stop old callback controller and any previous probe task.
old = getattr(builtins, "_echo_red_patrol_v1", None)
if old is not None:
    old.stop()
for key in ("_echo_red_motion_probe_v12", TASK_KEY):
    old_task = getattr(builtins, key, None)
    if old_task is not None and not old_task.done():
        old_task.cancel()

new_task = asyncio.ensure_future(run_test())
setattr(builtins, TASK_KEY, new_task)
print("ECHO M03B translation test scheduled; ARMED =", ARMED)
