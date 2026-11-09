#!/usr/bin/env python3
"""ECHO: Full warehouse, ROS Carter, red IRA NPCs and NavMesh."""

from pathlib import Path
import traceback

from isaacsim import SimulationApp

ROOT = Path.home() / "echo"
CONFIG = ROOT / "config/agents/ira/warehouse_two_npcs_one_human.yaml"
SCENE = ROOT / "scenes/full_warehouse/main_robot_ros.usd"

assert CONFIG.is_file(), f"Missing IRA config: {CONFIG}"
assert SCENE.is_file(), f"Missing warehouse scene: {SCENE}"

app = SimulationApp({"headless": False})

try:
    from isaacsim.core.utils.extensions import enable_extension

    extensions = [
        "isaacsim.ros2.bridge",
        "omni.anim.navigation.bundle",
        "isaacsim.replicator.agent.core",
        # ECHO development and debugging interface
        "omni.kit.window.script_editor",
        "omni.kit.window.console",
        "omni.kit.window.stage",
        "omni.kit.window.property",
        "omni.kit.window.extensions",
    ]

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
    import omni.anim.navigation.core as nav

    from isaacsim.replicator.agent.core import api as IRA

    async def setup_echo():
        print("[ECHO] Loading IRA configuration...", flush=True)

        if not IRA.load_config_file(str(CONFIG)):
            raise RuntimeError("IRA configuration could not be loaded")

        IRA.set_config(IRA.get_config_file())

        print("[ECHO] Creating warehouse and NPCs...", flush=True)
        await IRA.setup_simulation()

        kit = omni.kit.app.get_app()
        for _ in range(15):
            await kit.next_update_async()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("No USD stage after IRA setup")

        required = {
            "Main ROS Carter": "/World/Nova_Carter_ROS",
            "ROS clock": "/World/ROS_Clock",
            "Human agents root": "/World/ECHO_Humans",
            "NavMesh include": "/Navmesh/NavMeshIncludeVolume",
            "Red NPC 1": (
                "/ECHO_IRA_Robots/echo_npc_carters/Nova_Carter"
            ),
            "Red NPC 2": (
                "/ECHO_IRA_Robots/echo_npc_carters/Nova_Carter_01"
            ),
        }

        print("[ECHO] Checking scene components...", flush=True)

        for label, path in required.items():
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"Missing {label}: {path}")
            print(f"[ECHO] OK: {label}", flush=True)

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

        print("=" * 48, flush=True)
        print("[ECHO] SIMULATION READY - NOT PLAYING", flush=True)
        print("[ECHO] Main white ROS Carter: present", flush=True)
        print("[ECHO] Two red IRA NPCs: present", flush=True)
        print("[ECHO] Human IRA group: present", flush=True)
        print("[ECHO] NavMesh: baked", flush=True)
        print("[ECHO] Press Play in Isaac Sim to begin.", flush=True)
        print("=" * 48, flush=True)

    app.run_coroutine(setup_echo(), run_until_complete=True)

    while app.is_running():
        app.update()

except Exception:
    print("[ECHO] STARTUP FAILED", flush=True)
    traceback.print_exc()
    raise

finally:
    app.close()
