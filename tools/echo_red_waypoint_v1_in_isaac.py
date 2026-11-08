"""ECHO M03C: repeatable, simulation-time waypoint pilot for a No_Physics red Carter.

EXECUTE ONLY INSIDE ISAAC SIM SCRIPT EDITOR. No Nav2 publishers or control.
Starts DISARMED via config/agents/echo_red_waypoints_v1.json.
Only use after manually checking full swept robot footprint of every segment.
Uses the session USD layer and restores the initial pose on stop/completion.
Not collision safe; do not save stage while animated.
"""
import asyncio
import builtins
import csv
import json
import math
import sys
import time
from datetime import datetime
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Usd

ROOT = Path.home() / "echo"
CONFIG = ROOT / "config/agents/echo_red_waypoints_v1.json"
RESULTS = ROOT / "results/red_waypoint_v1"
REPORT = RESULTS / "red_waypoint_v1_report.json"
TASK_KEY = "_echo_red_waypoint_v1"
sys.path.insert(0, str(ROOT / "tools"))
from echo_waypoint_math import build_route, smoothstep


def write_report(data):
    RESULTS.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(data, indent=2), encoding="utf-8")


def apply_pose(stage, translation, rotation, pose):
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        translation.Set(Gf.Vec3d(*pose[:3]))
        rotation.Set(Gf.Vec3f(0.0, 0.0, float(pose[3])))


def interpolate(segment, u):
    w = smoothstep(u)
    xyz = tuple(a + (b-a)*w for a, b in zip(segment["from_xyz"], segment["to_xyz"]))
    yaw = segment["from_yaw"] + (segment["to_yaw"]-segment["from_yaw"])*w
    return xyz + (yaw,)


async def run():
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No Isaac Sim stage open")
    layer_id = stage.GetRootLayer().identifier
    if Path(layer_id).name != config["scene"]:
        raise RuntimeError(f"Expected {config['scene']}, got {Path(layer_id).name}")
    prim = stage.GetPrimAtPath(config["actor_prim"])
    if not prim.IsValid():
        raise RuntimeError("Configured actor does not exist")
    translation = prim.GetAttribute("xformOp:translate")
    rotation = prim.GetAttribute("xformOp:rotateXYZ")
    if not translation.IsValid() or not rotation.IsValid():
        raise RuntimeError("Missing xformOp:translate or xformOp:rotateXYZ")
    p, r = translation.Get(), rotation.Get()
    initial = (float(p[0]), float(p[1]), float(p[2]), float(r[2]))
    expected = config["expected_start_xyz_m"]
    tolerance = float(config["start_tolerance_m"])
    if not 0 < tolerance <= 0.1:
        raise ValueError("start_tolerance_m outside allowed range")
    if math.dist(initial[:3], expected) > tolerance:
        raise RuntimeError(f"Actor currently at {initial[:3]}, expected {expected}; refusing motion")
    segments, total_s, distance_m = build_route(config, initial)
    manifest = {
        "scene": layer_id, "actor_prim": config["actor_prim"],
        "start_pose_xyzyaw": initial, "segments": segments,
        "planned_distance_m": distance_m, "planned_duration_sim_s": total_s,
        "heading_mode": config["heading_mode"],
        "uses": "SimulationManager.get_simulation_time() + next_update_async()",
        "warning": "Visual-only USD animation; NO collision handling or guaranteed sensor detection",
    }
    if not config.get("armed", False):
        write_report({**manifest, "status": "NOT_ARMED",
                      "next_action": "Inspect the full swept footprint and set config armed=true only if safe"})
        print("ECHO M03C dry-run; NOT ARMED")
        print("Start:", initial, "segments:", len(segments),
              "path length:", round(distance_m, 3), "m",
              "duration:", round(total_s, 2), "sim seconds")
        for s in segments:
            print(s["kind"], "to", s["to_xyz"], "yaw", round(s["to_yaw"], 2),
                  "duration", round(s["duration_s"], 2))
        print("Manifest:", REPORT)
        return
    if not omni.timeline.get_timeline_interface().is_playing():
        raise RuntimeError("Isaac Sim must be playing before motion")

    # Refuse overlapping task controllers; parallel scripts could fight over USD poses.
    for key in ("_echo_red_motion_probe_v12", "_echo_red_translate_v13"):
        previous = getattr(builtins, key, None)
        if previous is not None and not previous.done():
            raise RuntimeError(f"Older actor controller is still active: {key}; stop it first")
    old_callback = getattr(builtins, "_echo_red_patrol_v1", None)
    if old_callback is not None:
        old_callback.stop()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = RESULTS / f"red_waypoint_v1_{stamp}.csv"
    status = "running"
    updates = 0
    physics_start = None
    last_sim = None
    stuck_updates = 0
    observed_distance = 0.0
    begin_wall = time.monotonic()
    readback_pose = initial
    started = False
    try:
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["update", "physics_sim_time_s", "elapsed_sim_s", "segment_index", "phase",
                             "x_m", "y_m", "z_m", "yaw_deg"])
            stream.flush()
            last_clock = None
            for _ in range(150):
                await omni.kit.app.get_app().next_update_async()
                current_stage = omni.usd.get_context().get_stage()
                if current_stage is None or current_stage.GetRootLayer().identifier != layer_id:
                    raise RuntimeError("Scene changed during preflight")
                clock = float(SimulationManager.get_simulation_time())
                if not math.isfinite(clock):
                    raise RuntimeError("Invalid physics time")
                if last_clock is not None and clock > last_clock + 1e-5:
                    physics_start = clock
                    break
                last_clock = clock
            if physics_start is None:
                raise RuntimeError("Physics time did not advance during preflight")
            last_sim = physics_start
            print("ECHO M03C armed, physics time confirmed. Starting waypoint run.")
            elapsed_limit = 0.0
            boundaries = []
            for index, segment in enumerate(segments):
                boundaries.append((elapsed_limit, elapsed_limit+segment["duration_s"], index, segment))
                elapsed_limit += segment["duration_s"]
            last_log = -1
            for frame in range(15000):
                await omni.kit.app.get_app().next_update_async()
                current_stage = omni.usd.get_context().get_stage()
                if current_stage is None or current_stage.GetRootLayer().identifier != layer_id:
                    raise RuntimeError("Scene changed; aborting")
                if not omni.timeline.get_timeline_interface().is_playing():
                    raise RuntimeError("Simulation paused; aborting")
                sim_time = float(SimulationManager.get_simulation_time())
                if not math.isfinite(sim_time) or sim_time < last_sim-0.001:
                    raise RuntimeError("Physics time invalid or jumped backward")
                if sim_time > last_sim + 1e-5:
                    stuck_updates = 0
                else:
                    stuck_updates += 1
                    if stuck_updates > 240:
                        raise RuntimeError("Physics time stalled for 240 application updates")
                last_sim = sim_time
                elapsed = max(0.0, sim_time-physics_start)
                if elapsed >= total_s:
                    pose = interpolate(segments[-1], 1.0)
                    phase = "finished"
                    idx = len(segments)-1
                else:
                    # For a bounded pilot, at most 16 segments: linear scan is sufficient.
                    for begin_s, end_s, idx, s in boundaries:
                        if elapsed < end_s:
                            break
                    u = (elapsed-begin_s)/(end_s-begin_s)
                    pose = interpolate(s, u)
                    phase = s["kind"]
                apply_pose(stage, translation, rotation, pose)
                pnew, rnew = translation.Get(), rotation.Get()
                readback_pose = (float(pnew[0]), float(pnew[1]), float(pnew[2]), float(rnew[2]))
                observed_distance = max(observed_distance, math.dist(readback_pose[:3], initial[:3]))
                updates += 1
                started = True
                if elapsed-last_log >= 0.1 or elapsed >= total_s:
                    writer.writerow([updates, f"{sim_time:.6f}", f"{elapsed:.6f}", idx, phase,
                                     f"{readback_pose[0]:.6f}", f"{readback_pose[1]:.6f}",
                                     f"{readback_pose[2]:.6f}", f"{readback_pose[3]:.4f}"])
                    stream.flush()
                    last_log = elapsed
                if elapsed >= total_s:
                    status = "complete"
                    break
                if time.monotonic()-begin_wall > 240:
                    raise RuntimeError("Wall-time watchdog exceeded 240s")
            else:
                raise RuntimeError("Exceeded 15,000 app updates")
    except asyncio.CancelledError:
        status = "cancelled"
        raise
    except Exception as exc:
        status = f"error: {exc!r}"
        print("ECHO M03C:", status)
    finally:
        restored = False
        try:
            current_stage = omni.usd.get_context().get_stage()
            if current_stage is not None and current_stage.GetRootLayer().identifier == layer_id:
                apply_pose(stage, translation, rotation, initial)
                post, rot = translation.Get(), rotation.Get()
                restored = math.dist(tuple(float(x) for x in post), initial[:3]) < 1e-6 and abs(float(rot[2])-initial[3]) < 1e-5
        except Exception as exc:
            status += f"; restore error {exc!r}"
        write_report({**manifest, "status": status, "updates": updates,
                      "physics_time_start_s": physics_start, "physics_time_end_s": last_sim,
                      "physics_time_span_s": None if last_sim is None or physics_start is None else last_sim-physics_start,
                      "max_displacement_from_start_m": observed_distance,
                      "last_readback_xyzyaw": readback_pose,
                      "restored_initial_pose": restored,
                      "wall_elapsed_s": round(time.monotonic()-begin_wall, 3),
                      "csv_path": str(csv_path), "motion_started": started})
        print("ECHO M03C", status, "restored:", restored, "report:", REPORT)


# Only schedule one waypoint pilot at a time.
existing = getattr(builtins, TASK_KEY, None)
if existing is not None and not existing.done():
    raise RuntimeError("Waypoint pilot already running; use stop script before restarting")
task = asyncio.ensure_future(run())
setattr(builtins, TASK_KEY, task)
print("ECHO M03C waypoint pilot scheduled; open the JSON report for status")
