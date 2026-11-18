"""ECHO M04B: IRA agent ground-truth logger.

Run inside Isaac Sim's Script Editor while playing.
Read-only: does not modify any actors or USD assets.
"""

import asyncio
import csv
import hashlib
import subprocess
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import omni.kit.app
import omni.timeline
import omni.usd
from omni.metropolis.pipeline.agent import AgentsManager

ROOT = Path.home() / "echo"
OUT = ROOT / "results/m04_ground_truth"

RATE_HZ = 10.0
DURATION_S = 15.0


async def echo_log_ira_ground_truth():
    timeline = omni.timeline.get_timeline_interface()

    if not timeline.is_playing():
        print("[ECHO GT] Press Play first.")
        return

    manager = AgentsManager.get_instance()
    agents = list(manager.get_runtime_agent_instances())

    print("\n=== ECHO M04B GROUND TRUTH LOGGER ===")
    print("Runtime agents:", len(agents))

    if len(agents) != 3:
        print("Expected 3 agents. Aborting.")
        return

    identities = []
    seen_names = set()

    for index, agent in enumerate(agents):
        kind = type(agent).__name__
        actor_id = str(agent.name)
        prim_path = str(agent.prim_path)

        if not actor_id or actor_id in seen_names:
            print("[ECHO GT] Invalid or duplicate actor name.")
            return

        seen_names.add(actor_id)

        identities.append(
            (actor_id, prim_path, kind, index, agent)
        )

        print(f"  {actor_id}: {kind}")
        print(f"    USD path: {prim_path}")

    OUT.mkdir(parents=True, exist_ok=True)

    run_id = datetime.now(timezone.utc).strftime(
        "%Y%m%dT%H%M%SZ"
    )

    csv_path = OUT / f"m04b_{run_id}.csv"
    meta_path = OUT / f"m04b_{run_id}.json"

    fields = [
        "sim_time_s",
        "actor_id",
        "prim_path",
        "runtime_index",
        "actor_type",
        "x_m", "y_m", "z_m",
        "rot_0", "rot_1", "rot_2", "rot_3",
        "yaw_raw_rad",
        "omega_z_radps",
        "reported_speed",
        "vx_mps", "vy_mps",
        "measured_speed_mps",
        "task",
    ]

    def optional_value(agent, method):
        try:
            return getattr(agent, method)()
        except Exception:
            return None

    previous = {}
    sample_times = []
    sample_count = 0
    row_count = 0

    app = omni.kit.app.get_app()
    start_time = float(timeline.get_current_time())
    last_time = None
    end_time = start_time

    print("Start sim time:", round(start_time, 3))
    print("Output:", csv_path)

    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        while timeline.is_playing():
            await app.next_update_async()

            t = float(timeline.get_current_time())

            if t < start_time:
                print("[ECHO GT] Timeline reset. Stopping.")
                break

            if last_time is not None:
                if t - last_time < (1.0 / RATE_HZ - 1e-6):
                    continue

            last_time = t
            end_time = t
            sample_count += 1
            sample_times.append(t)

            for actor_id, prim_path, kind, index, agent in identities:
                p = agent.get_world_position()
                pos = tuple(float(p[i]) for i in range(3))

                rot = optional_value(
                    agent, "get_world_rotation"
                )

                try:
                    rot_values = [
                        float(rot[i]) for i in range(4)
                    ]
                except Exception:
                    rot_values = [""] * 4

                reported = optional_value(agent, "get_speed")
                task = optional_value(
                    agent, "get_current_task_name"
                )

                vx = vy = measured_speed = ""
                yaw_raw_rad = ""
                omega_z_radps = ""

                if all(
                    isinstance(v, float) and math.isfinite(v)
                    for v in rot_values
                ):
                    x, y, z, w = rot_values
                    q_norm = math.sqrt(
                        x*x + y*y + z*z + w*w
                    )

                    if 0.95 <= q_norm <= 1.05:
                        x /= q_norm
                        y /= q_norm
                        z /= q_norm
                        w /= q_norm

                        yaw_raw_rad = math.atan2(
                            2.0 * (w*z + x*y),
                            1.0 - 2.0 * (y*y + z*z)
                        )

                if actor_id in previous:
                    prev_t, prev_p, prev_yaw = previous[actor_id]
                    dt = t - prev_t

                    if dt > 0:
                        vx = (pos[0] - prev_p[0]) / dt
                        vy = (pos[1] - prev_p[1]) / dt
                        measured_speed = math.hypot(vx, vy)

                        if yaw_raw_rad != "" and prev_yaw != "":
                            delta = yaw_raw_rad - prev_yaw
                            delta = math.atan2(
                                math.sin(delta),
                                math.cos(delta)
                            )
                            omega_z_radps = delta / dt

                previous[actor_id] = (
                    t, pos, yaw_raw_rad
                )

                writer.writerow({
                    "sim_time_s": t,
                    "actor_id": actor_id,
                    "prim_path": prim_path,
                    "runtime_index": index,
                    "actor_type": kind,
                    "x_m": pos[0],
                    "y_m": pos[1],
                    "z_m": pos[2],
                    "rot_0": rot_values[0],
                    "rot_1": rot_values[1],
                    "rot_2": rot_values[2],
                    "rot_3": rot_values[3],
                    "yaw_raw_rad": yaw_raw_rad,
                    "omega_z_radps": omega_z_radps,
                    "reported_speed": (
                        reported if reported is not None else ""
                    ),
                    "vx_mps": vx,
                    "vy_mps": vy,
                    "measured_speed_mps": measured_speed,
                    "task": str(task) if task is not None else "",
                })

                row_count += 1

            f.flush()

            if t - start_time >= DURATION_S:
                break

    intervals = [
        b - a for a, b in zip(sample_times, sample_times[1:])
    ]

    actual_rate = (
        (len(sample_times) - 1) /
        (sample_times[-1] - sample_times[0])
        if len(sample_times) > 1
        and sample_times[-1] > sample_times[0]
        else None
    )

    try:
        git_revision = subprocess.check_output(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            text=True,
            stderr=subprocess.DEVNULL
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        git_revision = None

    config_path = ROOT / (
        "config/agents/ira/"
        "warehouse_two_npcs_one_human.yaml"
    )

    config_hash = (
        hashlib.sha256(config_path.read_bytes()).hexdigest()
        if config_path.is_file() else None
    )

    stage = omni.usd.get_context().get_stage()
    scene_root = (
        str(stage.GetRootLayer().identifier)
        if stage else None
    )

    metadata = {
        "schema_version": 1,
        "source": "IRA runtime simulator ground truth",
        "git_commit": git_revision,
        "scene_root_layer": scene_root,
        "config_sha256": config_hash,
        "actual_rate_hz": actual_rate,
        "first_sample_sim_time_s": (
            sample_times[0] if sample_times else None
        ),
        "last_sample_sim_time_s": (
            sample_times[-1] if sample_times else None
        ),
        "sample_interval_min_s": (
            min(intervals) if intervals else None
        ),
        "sample_interval_max_s": (
            max(intervals) if intervals else None
        ),
        "rotation_convention": "xyzw",
        "experiment": "ECHO_M04B",
        "run_id": run_id,
        "target_rate_hz": RATE_HZ,
        "target_duration_s": DURATION_S,
        "start_sim_time_s": start_time,
        "end_sim_time_s": end_time,
        "samples": sample_count,
        "rows": row_count,
        "actor_ids": [a[0] for a in identities],
        "actor_paths": {a[0]: a[1] for a in identities},
        "config": (
            "config/agents/ira/"
            "warehouse_two_npcs_one_human.yaml"
        ),
        "notes": (
            "IRA simulator ground truth. Actor IDs are "
            "reproducible in the current scene across tested restarts. "
            "Robot quaternion XYZW convention verified; human visual "
            "facing-axis offset remains uncalibrated. Reported speed "
            "may differ from velocity measured from positions."
        ),
    }

    meta_path.write_text(json.dumps(metadata, indent=2))

    print("\n=== ECHO GROUND TRUTH COMPLETE ===")
    print("Samples:", sample_count)
    print("Rows:", row_count)
    print(
        "Simulation duration:",
        round(end_time - start_time, 3)
    )
    print("CSV:", csv_path)
    print("Metadata:", meta_path)


asyncio.ensure_future(echo_log_ira_ground_truth())
