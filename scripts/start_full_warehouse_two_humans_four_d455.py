#!/usr/bin/env python3
"""ECHO: Full warehouse, ROS Carter, red IRA NPCs and NavMesh."""

from pathlib import Path
import os
import sys
import traceback

from isaacsim import SimulationApp

ROOT = Path.home() / "echo"
CONFIG = ROOT / "config/agents/ira/warehouse_two_npcs_two_humans_four_d455.yaml"
SCENE = ROOT / "scenes/s01_d455/s01_d455_four_rgbd_raw_trial.usda"

AGENTS = os.environ.get("ECHO_S04_AGENTS", "dynamic")
if AGENTS not in ("dynamic", "none"):
    raise ValueError(f"Unknown agent mode: {AGENTS}")
if AGENTS == "dynamic":
    assert CONFIG.is_file(), f"Missing IRA config: {CONFIG}"
assert SCENE.is_file(), f"Missing warehouse scene: {SCENE}"

FIXTURE_DIR = os.environ.get("ECHO_S04_OBSTACLE_DIR")
if FIXTURE_DIR and AGENTS != "none":
    raise RuntimeError("S04 obstacle fixture requires clean agent mode")
POSE_DIR = os.environ.get("ECHO_S04_POSE_DIR")
if POSE_DIR and FIXTURE_DIR:
    raise RuntimeError("Select either obstacle fixture or pose-only logging")
fixture = None

app = SimulationApp({"headless": False})

try:
    from isaacsim.core.utils.extensions import enable_extension

    extensions = [
        "isaacsim.ros2.bridge",
        # ECHO development and debugging interface
        "omni.kit.window.script_editor",
        "omni.kit.window.console",
        "omni.kit.window.stage",
        "omni.kit.window.property",
        "omni.kit.window.extensions",
    ]

    if AGENTS == "dynamic":
        extensions[1:1] = ["omni.anim.navigation.bundle", "isaacsim.replicator.agent.core"]

    for extension in extensions:
        print(f"[ECHO] Enabling: {extension}", flush=True)
        enable_extension(extension)
        app.update()

    import omni.kit.app
    import omni.ui

    # Open Script Editor automatically.
    omni.ui.Workspace.show_window("Script Editor", True)
    app.update()

    import omni.timeline
    import omni.usd
    if AGENTS == "dynamic":
        import omni.anim.navigation.core as nav
        from isaacsim.replicator.agent.core import api as IRA

    async def setup_echo():
        global fixture
        if AGENTS == "dynamic":
            print("[ECHO] Loading IRA configuration...", flush=True)

            if not IRA.load_config_file(str(CONFIG)):
                raise RuntimeError("IRA configuration could not be loaded")

            IRA.set_config(IRA.get_config_file())

            print("[ECHO] Creating warehouse and NPCs...", flush=True)
            await IRA.setup_simulation()
        else:
            print("[ECHO] Opening original four-D455 USD without IRA", flush=True)
            success, error = await omni.usd.get_context().open_stage_async(str(SCENE))
            if not success:
                raise RuntimeError(f"Clean stage open failed: {error}")

        kit = omni.kit.app.get_app()
        for _ in range(15):
            await kit.next_update_async()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No USD stage after scene setup")

        required = {
            "Main ROS Carter": "/World/Nova_Carter_ROS",
            "ROS clock": "/World/ROS_Clock",
        }
        actor_paths = {
            "Human agents root": "/World/ECHO_Humans",
            "Red robot root": "/ECHO_IRA_Robots",
        }
        if AGENTS == "dynamic":
            required.update({
                "Human agents root": actor_paths["Human agents root"],
                "NavMesh include": "/Navmesh/NavMeshIncludeVolume",
                "Red NPC 1": "/ECHO_IRA_Robots/echo_npc_carters/Nova_Carter",
                "Red NPC 2": "/ECHO_IRA_Robots/echo_npc_carters/Nova_Carter_01",
            })
        else:
            for label, path in actor_paths.items():
                prim = stage.GetPrimAtPath(path)
                if prim.IsValid() and prim.IsActive():
                    raise RuntimeError(f"Unexpected active actor root: {label}: {path}")
            print("[ECHO] Known IRA actor roots absent or inactive", flush=True)

        print("[ECHO] Checking scene components...", flush=True)

        for label, path in required.items():
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Missing {label}: {path}")
            print(f"[ECHO] OK: {label}", flush=True)

        print("[ECHO] Checking four D455 cameras...", flush=True)

        for side in ("front", "rear", "left", "right"):
            camera_path = (
                "/World/Nova_Carter_ROS/chassis_link/sensors/"
                f"echo_d455_{side}/RSD455"
            )
            prim = stage.GetPrimAtPath(camera_path)

            if not prim.IsValid() or not prim.IsActive():
                raise RuntimeError(
                    f"Missing or inactive {side} D455: {camera_path}"
                )

            print(f"[ECHO] OK: {side} D455", flush=True)

        if AGENTS == "dynamic":
            human_root = stage.GetPrimAtPath("/World/ECHO_Humans")
            human_children = list(human_root.GetChildren())
            print(
                "[ECHO] Human root children:",
                [str(p.GetPath()) for p in human_children],
                flush=True,
            )
            if not human_children:
                raise RuntimeError("No human group created by IRA")

            print("[ECHO] Baking NavMesh...", flush=True)

            nav_interface = nav.acquire_interface()
            nav_interface.start_navmesh_baking_and_wait()

            for _ in range(5):
                await kit.next_update_async()

            if nav_interface.get_navmesh() is None:
                raise RuntimeError("NavMesh baking did not produce a mesh")

            print("[ECHO] NavMesh ready", flush=True)
        else:
            print("[ECHO] Clean mode: IRA setup and NavMesh baking skipped", flush=True)

        # Set a usable one-hour timeline range in memory.
        # Do not save the generated NPCs into the base USD.
        if stage.GetEndTimeCode() <= stage.GetStartTimeCode():
            stage.SetStartTimeCode(0.0)
            stage.SetEndTimeCode(
                stage.GetTimeCodesPerSecond() * 3600.0
            )
            print("[ECHO] Runtime timeline range configured", flush=True)

        timeline = omni.timeline.get_timeline_interface()
        timeline.set_looping(False)

        # Do not automatically start physics or NPC movement.
        if timeline.is_playing():
            timeline.pause()

        timeline.set_current_time(0.0)

        for _ in range(5):
            await kit.next_update_async()

        if timeline.is_playing():
            raise RuntimeError("Simulation unexpectedly started")

        if FIXTURE_DIR:
            sys.path.insert(0, str(ROOT / "tools/s04"))
            from obstacle_fixture import ObstacleFixture
            fixture = ObstacleFixture(stage, timeline, FIXTURE_DIR)
        elif POSE_DIR:
            sys.path.insert(0, str(ROOT / "tools/s04"))
            from pose_logger import PoseLogger
            fixture = PoseLogger(stage, timeline, POSE_DIR)

        print("=" * 48, flush=True)
        print("[ECHO] SIMULATION READY - NOT PLAYING", flush=True)
        print("[ECHO] Main white ROS Carter: present", flush=True)
        if AGENTS == "dynamic":
            print("[ECHO] Two red IRA NPCs: present", flush=True)
            print("[ECHO] Human IRA group: present", flush=True)
            print("[ECHO] NavMesh: baked", flush=True)
        else:
            print("[ECHO] Clean warehouse: no IRA agents spawned", flush=True)
        print("[ECHO] Press Play in Isaac Sim to begin.", flush=True)
        print("=" * 48, flush=True)

    app.run_coroutine(setup_echo(), run_until_complete=True)

    while app.is_running():
        app.update()
        if fixture is not None:
            fixture.sample()

except Exception:
    print("[ECHO] STARTUP FAILED", flush=True)
    traceback.print_exc()
    raise

finally:
    if fixture is not None:
        fixture.close()
    app.close()
