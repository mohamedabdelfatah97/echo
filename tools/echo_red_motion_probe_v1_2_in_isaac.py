"""ECHO M03: bounded red-Carter rotation test using async Kit app updates.

Run from Isaac Sim Script Editor only. This is a *visual-only* test that uses
application-frame count, deliberately not simulation time, to isolate update
callbacks from timeline problems. It is NOT a valid simulation-time trajectory
or collision-safe controller. Never save the modified USD stage.
"""
import asyncio
import builtins
import csv
import json
import time
from datetime import datetime
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Usd

ROOT = Path.home() / "echo"
RESULTS = ROOT / "results" / "red_patrol_v1"
SCENE = "echo_warehouse_dynamic_red_patrol_v1.usda"
ACTOR = "/World/ECHO_Dynamics/Robots/Red_Carter"
TASK_KEY = "_echo_red_motion_probe_v12"


def smoothstep(t):
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


async def probe():
    stage = omni.usd.get_context().get_stage()
    if stage is None or Path(stage.GetRootLayer().identifier).name != SCENE:
        raise RuntimeError(f"Expected the separate experimental scene: {SCENE}")
    root_layer = stage.GetRootLayer().identifier
    prim = stage.GetPrimAtPath(ACTOR)
    if not prim.IsValid():
        raise RuntimeError(f"Actor not found: {ACTOR}")
    rotation = prim.GetAttribute("xformOp:rotateXYZ")
    if not rotation.IsValid():
        raise RuntimeError("Missing red Carter xformOp:rotateXYZ")
    old_rotation = rotation.Get()
    initial = tuple(float(v) for v in old_rotation)
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        timeline.commit()

    RESULTS.mkdir(parents=True, exist_ok=True)
    label = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = RESULTS / f"red_motion_probe_v12_{label}.csv"
    report_path = RESULTS / "red_motion_probe_v12_report.json"
    t0 = time.monotonic()
    samples = 0
    max_delta = 0.0
    timeline_start = float(timeline.get_current_time())
    timeline_min = timeline_start
    timeline_max = timeline_start
    status = "running"

    # Two 60-update ramps with a 240-update hold in between.
    # This deliberately tests Kit app updates even if sim time is stuck at zero.
    try:
        with csv_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(["app_frame", "wall_elapsed_s", "timeline_s", "playing", "commanded_yaw_deg", "readback_yaw_deg"])
            output.flush()
            for i in range(361):
                await omni.kit.app.get_app().next_update_async()
                current_stage = omni.usd.get_context().get_stage()
                if current_stage is None or current_stage.GetRootLayer().identifier != root_layer:
                    raise RuntimeError("Stage changed during test; aborting")

                if i <= 60:
                    fraction = smoothstep(i / 60)
                elif i <= 300:
                    fraction = 1.0
                else:
                    fraction = 1.0 - smoothstep((i - 300) / 60)

                requested_yaw = initial[2] + 50.0 * fraction
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    rotation.Set(Gf.Vec3f(initial[0], initial[1], requested_yaw))
                readback = float(rotation.Get()[2])
                now_sim = float(timeline.get_current_time())
                timeline_min = min(timeline_min, now_sim)
                timeline_max = max(timeline_max, now_sim)
                max_delta = max(max_delta, abs(readback - initial[2]))
                samples += 1
                if i % 10 == 0:
                    writer.writerow([i, f"{time.monotonic()-t0:.3f}", f"{now_sim:.6f}", timeline.is_playing(), f"{requested_yaw:.3f}", f"{readback:.3f}"])
                    output.flush()
                if i % 20 == 0:
                    report_path.write_text(json.dumps({
                        "status": "running", "updates": samples,
                        "timeline_start_s": timeline_start,
                        "timeline_end_s": now_sim,
                        "max_readback_yaw_delta_deg": max_delta,
                        "csv_path": str(csv_path)
                    }, indent=2))
            status = "complete"
    except asyncio.CancelledError:
        status = "cancelled"
        raise
    except Exception as exc:
        status = f"error: {exc!r}"
        print("ECHO M03 probe error:", repr(exc))
    finally:
        # Restore only the red experimental actor; do not touch the main robot.
        try:
            current_stage = omni.usd.get_context().get_stage()
            if current_stage is not None and current_stage.GetRootLayer().identifier == root_layer:
                with Usd.EditContext(stage, stage.GetSessionLayer()):
                    rotation.Set(Gf.Vec3f(*initial))
        except Exception as exc:
            status += f"; restore failed: {exc!r}"
        report = {
            "status": status,
            "updates": samples,
            "timeline_playing_at_end": timeline.is_playing(),
            "timeline_start_s": timeline_start,
            "timeline_min_s": timeline_min,
            "timeline_max_s": timeline_max,
            "timeline_span_s": timeline_max - timeline_min,
            "wall_elapsed_s": round(time.monotonic() - t0, 3),
            "max_readback_yaw_delta_deg": max_delta,
            "csv_path": str(csv_path),
            "warning": "App-frame visual motion, not simulation-timed ground truth; no collision avoidance."
        }
        report_path.write_text(json.dumps(report, indent=2))
        print("ECHO M03 probe report:", report_path)
        print("ECHO M03 probe status:", status, "updates:", samples)


# Stop the old v1/v1.1 callback controller, if it is still running.
old = getattr(builtins, "_echo_red_patrol_v1", None)
if old is not None:
    old.stop()

previous = getattr(builtins, TASK_KEY, None)
if previous is not None and not previous.done():
    previous.cancel()

# Keep a reference so the task can be examined or cancelled from Script Editor.
task = asyncio.ensure_future(probe())
setattr(builtins, TASK_KEY, task)
print("ECHO M03: async 50-degree visual motion probe scheduled; do not save the scene.")
print("Report: ~/echo/results/red_patrol_v1/red_motion_probe_v12_report.json")
