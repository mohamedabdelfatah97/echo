#!/usr/bin/env python3
"""ECHO dynamic four-D455 stack supervisor; never edits USD or ROS configs."""
import argparse
import fcntl
import os
import shlex
import signal
import subprocess
import tempfile
import time
from pathlib import Path

ROOT = Path.home() / "echo"
ROS_WS = Path.home() / "IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash"
DETECT = (
    "start_echo_d455_four_live.sh",
    "start_full_warehouse_two_humans_four_d455.sh",
    "start_full_warehouse_two_humans_four_d455.py",
    "start_echo_d455_front_helpers.sh",
    "start_echo_d455_rear_helpers.sh",
    "start_echo_d455_side_helpers.sh",
    "echo_d455_front_static_tf.py",
    "echo_d455_rear_static_tf.py",
    "echo_d455_left_static_tf.py",
    "echo_d455_right_static_tf.py",
    "echo_d455_front_depth_camera_info.py",
    "echo_d455_rear_depth_camera_info.py",
    "echo_d455_left_depth_camera_info.py",
    "echo_d455_right_depth_camera_info.py",
    "echo_d455_",  # includes existing cloud converters and related ECHO nodes
    "echo_s03_four_rgbd_sync_probe.launch.py",
    "echo_s03_",  # includes orphaned S03 sync/register nodes
    "isaacsim.exp.full.kit",
)


def existing():
    matches = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            args = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except (OSError, PermissionError):
            continue
        if any(word in args for word in DETECT):
            matches.append((int(entry.name), args[:180]))
    return sorted(matches)


def ros_command(argv):
    prefix = (
        f"source /opt/ros/jazzy/setup.bash && "
        f"source {shlex.quote(str(ROS_WS))} && "
        "export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp && exec "
    )
    return ["bash", "-c", prefix + shlex.join(argv)]


def group_alive(pid):
    try:
        os.killpg(pid, 0)
        return True
    except ProcessLookupError:
        return False


def stop_groups(processes):
    for sig, delay in ((signal.SIGINT, 3), (signal.SIGTERM, 5), (signal.SIGKILL, 1)):
        for name, proc in reversed(processes):
            if group_alive(proc.pid):
                try:
                    os.killpg(proc.pid, sig)
                except ProcessLookupError:
                    pass
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            if not any(group_alive(proc.pid) for _, proc in processes):
                break
            time.sleep(0.2)
    for _, proc in processes:
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Read-only preflight; never starts processes")
    parser.add_argument("--calibration", choices=("baseline", "coupled_xy"),
                        default="baseline", help="Select S03 CameraInfo consumer mode")
    parser.add_argument("--agents", choices=("dynamic", "none"), default="dynamic",
                        help="S04: existing IRA actors or clean warehouse")
    args = parser.parse_args()

    # The historic depth-only experiment is incompatible with a coupled mode.
    legacy = os.environ.get("ECHO_S03_DEPTH_CY_MODE", "baseline").strip().lower()
    if legacy != "baseline":
        parser.error("Set ECHO_S03_DEPTH_CY_MODE=baseline; use --calibration coupled_xy instead")
    info_root = ("/echo/s03/calibration/coupled_xy" if args.calibration == "coupled_xy"
                 else "/echo/cameras")
    child_env = dict(os.environ, ECHO_S03_DEPTH_CY_MODE="baseline",
                     ECHO_S03_CAMERA_INFO_ROOT=info_root,
                     ECHO_S04_AGENTS=args.agents)
    print(f"[ECHO] Agent mode: {args.agents}", flush=True)
    print(f"[ECHO] Calibration: {args.calibration}; consumer CameraInfo root: {info_root}", flush=True)

    required = [
        ROOT / "scripts/start_full_warehouse_two_humans_four_d455.sh",
        ROOT / "scripts/start_echo_d455_front_helpers.sh",
        ROOT / "scripts/start_echo_d455_rear_helpers.sh",
        ROOT / "scripts/start_echo_d455_side_helpers.sh",
        ROOT / "launch/echo_s03_four_rgbd_sync_probe.launch.py",
        ROS_WS,
    ]
    relay = ROOT / "scripts/echo_s03_coupled_xy_camera_info.py"
    if args.calibration == "coupled_xy":
        required.append(relay)
    missing = [str(path) for path in required if not path.is_file()]
    occupied = existing()
    print("[ECHO] Required files:", "OK" if not missing else "MISSING", flush=True)
    for item in missing:
        print("  MISSING:", item, flush=True)
    print("[ECHO] Existing experiment processes:", len(occupied), flush=True)
    for pid, cmd in occupied:
        print(f"  {pid}: {cmd}", flush=True)
    if args.check:
        return 0 if not missing else 1
    if missing or occupied:
        print("[ECHO] Refusing to start: close the current experiment first.", flush=True)
        return 1

    lock = open("/tmp/echo_dynamic_four_d455_supervisor.lock", "a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("[ECHO] Another supervisor is already running.", flush=True)
        return 1

    logs = Path(tempfile.mkdtemp(prefix="echo_dynamic_four_d455_"))
    processes = []
    stopping = False

    def on_signal(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def start(name, argv):
        log_file = logs / f"{name}.log"
        with log_file.open("w") as handle:
            proc = subprocess.Popen(
                ros_command(argv), cwd=ROOT, stdout=handle,
                stderr=subprocess.STDOUT, start_new_session=True, env=child_env,
            )
        processes.append((name, proc))
        print(f"[ECHO] Started {name}: PID/PGID {proc.pid}; log {log_file}", flush=True)
        return proc

    try:
        sim = start("sim", ["bash", str(required[0])])
        ready = False
        until = time.monotonic() + 180
        while time.monotonic() < until and not stopping:
            if sim.poll() is not None:
                raise RuntimeError(f"Isaac Sim exited early ({sim.returncode})")
            if "[ECHO] SIMULATION READY - NOT PLAYING" in (logs / "sim.log").read_text(errors="replace"):
                ready = True
                break
            time.sleep(0.5)
        if stopping:
            return 130
        if not ready:
            raise RuntimeError("Isaac Sim did not report readiness; inspect sim.log")

        print("[ECHO] Scene ready; starting ROS helpers and S03 sync.", flush=True)
        for name, filename in (
            ("front_helpers", "start_echo_d455_front_helpers.sh"),
            ("rear_helpers", "start_echo_d455_rear_helpers.sh"),
            ("side_helpers", "start_echo_d455_side_helpers.sh"),
        ):
            start(name, ["bash", str(ROOT / "scripts" / filename)])

        if args.calibration == "coupled_xy":
            start("coupled_xy_info", ["python3", str(relay)])

        start("front_cloud", [
            "ros2", "run", "depth_image_proc", "point_cloud_xyz_node", "--ros-args",
            "-r", "__node:=echo_d455_front_depth_to_cloud",
            "-r", "image_rect:=/echo/cameras/front/depth/image_raw",
            "-r", f"/echo/cameras/front/depth/camera_info:={info_root}/front/depth/camera_info",
            "-r", "points:=/echo/cameras/front/depth/points",
        ])
        time.sleep(2)
        start("s03_sync", ["ros2", "launch", str(required[4]),
                           f"camera_info_root:={info_root}"])
        print("[ECHO] Press Play in Isaac Sim. Ctrl+C stops only this managed stack.", flush=True)
        print(f"[ECHO] Logs: {logs}", flush=True)
        while not stopping:
            for name, proc in processes:
                if proc.poll() is not None:
                    raise RuntimeError(f"{name} exited unexpectedly ({proc.returncode}); inspect {logs / (name + '.log')}")
            time.sleep(0.5)
        return 0
    except Exception as exc:
        print(f"[ECHO] ERROR: {exc}", flush=True)
        print(f"[ECHO] Logs: {logs}", flush=True)
        return 1
    finally:
        print("[ECHO] Stopping managed process groups...", flush=True)
        stop_groups(processes)
        print("[ECHO] Shutdown complete.", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
