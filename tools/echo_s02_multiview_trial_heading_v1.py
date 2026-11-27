#!/usr/bin/env python3
"""ECHO S02: bounded, human-supervised RGB-D multiview mapping trial.

Runs the existing isolated range-10 RTAB-Map launcher, commands a conservative
odometry-limited route, records lightweight ROS metrics, and inspects the saved
RTAB-Map grid geometry. Does NOT provide collision avoidance or ground truth.

Run with --execute only after visually verifying >1.5 m clear aisle ahead,
no approaching agents, and no Nav2/other /cmd_vel publishers.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import sys
import time
import traceback
import zlib


NODE_NAME = "echo_s02_multiview_trial"
CMD_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/chassis/odom"
INFO_TOPIC = "/echo_s02_route/info"
GRAPH_TOPIC = "/echo_s02_route/mapGraph"
CLOUD_TOPIC = "/echo_s02_route/cloud_map"
REG_TOPIC = "/echo/cameras/front/depth_registered/image_rect"


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds")


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def get_yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def format_mib(path: Path) -> float:
    try:
        return path.stat().st_size / (1024 * 1024)
    except FileNotFoundError:
        return 0.0


def parse_args():
    p = argparse.ArgumentParser(
        description="Bounded ECHO S02 RTAB-Map multi-view trial + logger."
    )
    p.add_argument("--execute", action="store_true",
                   help="Actually start RTAB-Map and command the route; otherwise preview only")
    p.add_argument("--repo", type=Path, default=Path.home() / "echo")
    p.add_argument("--output-root", type=Path, default=Path.home() / "echo" / "diagnostics" / "s02")
    p.add_argument("--leg-m", type=float, default=0.35,
                   help="Length of each of three forward segments, 0.10 to 0.50 m")
    p.add_argument("--forward-mps", type=float, default=0.055)
    p.add_argument("--turn-radps", type=float, default=0.14)
    p.add_argument("--yaw-deg", type=float, default=18.0,
                   help="Turn +A degrees, then -2A, then +A")
    p.add_argument("--max-db-mib", type=float, default=350.0)
    p.add_argument("--max-wall-s", type=float, default=150.0)
    p.add_argument("--warmup-s", type=float, default=25.0)
    p.add_argument("--analyze", type=Path, help="Only analyze an existing RTAB-Map .db (read-only)")
    return p.parse_args()


def validate_options(a):
    if not 0.10 <= a.leg_m <= 0.50:
        raise ValueError("--leg-m must be 0.10 to 0.50")
    if not 0.02 <= a.forward_mps <= 0.10:
        raise ValueError("--forward-mps must be 0.02 to 0.10")
    if not 0.07 <= a.turn_radps <= 0.30:
        raise ValueError("--turn-radps must be 0.07 to 0.30")
    if not 5.0 <= a.yaw_deg <= 30.0:
        raise ValueError("--yaw-deg must be 5 to 30")
    if a.max_db_mib < 50 or a.max_wall_s < 50:
        raise ValueError("Resource limits are too small to run a useful mapping test")


def plan(a):
    # The net route is a shallow zig-zag rather than a straight line only.
    return [
        ("forward_1", "forward", a.leg_m),
        ("turn_left", "turn", a.yaw_deg),
        ("forward_2", "forward", a.leg_m),
        ("turn_right", "turn", -2 * a.yaw_deg),
        ("forward_3", "forward", a.leg_m),
        ("restore_heading", "turn", a.yaw_deg),
    ]


def is_s02_process_present() -> list[str]:
    """Read-only process check; never broad-kill an existing process."""
    try:
        text = subprocess.check_output(
            ["ps", "-eo", "pid=,args="], text=True, timeout=3
        )
    except Exception:
        return ["Unable to inspect process list"]
    matches = []
    for line in text.splitlines():
        if ("/rtabmap_slam/rtabmap" in line or
                "/rtabmap_sync/rgbd_sync" in line or
                "/depth_image_proc/register_node" in line and
                "echo_s02" in line):
            matches.append(line.strip())
    return matches


def analyze_db(db: Path) -> dict:
    """Read-only and bounded: inspect a handful of the most recent stored grids."""
    report = {"path": str(db), "size_mib": round(format_mib(db), 1)}
    if not db.is_file():
        report["status"] = "database_not_found"
        return report
    try:
        import numpy as np
        with sqlite3.connect(f"file:{db}?mode=ro", uri=True, timeout=4) as conn:
            conn.execute("PRAGMA query_only=ON")
            for tbl in ("Node", "Link", "Data"):
                report[f"{tbl.lower()}_count"] = int(conn.execute(
                    f"SELECT count(*) FROM {tbl}").fetchone()[0])
            report["retained_weight_nonnegative"] = int(conn.execute(
                "SELECT count(*) FROM Node WHERE weight >= 0").fetchone()[0])
            rows = conn.execute("""
                SELECT id, ground_cells, obstacle_cells FROM Data
                WHERE ground_cells IS NOT NULL AND obstacle_cells IS NOT NULL
                ORDER BY id DESC LIMIT 6
            """).fetchall()
        samples = []
        for node_id, ground, obstacle in rows:
            entry = {"node_id": node_id}
            for name, blob in (("ground", ground), ("obstacle", obstacle)):
                if not blob or len(blob) < 13:
                    entry[name] = {"status": "no_data"}
                    continue
                height, width, cv_type = struct.unpack("<iii", blob[-12:])
                channels, depth = (cv_type >> 3) + 1, cv_type & 7
                if not (depth == 5 and channels >= 3):
                    entry[name] = {"status": "unsupported", "cv_type": cv_type}
                    continue
                raw = zlib.decompress(blob[:-12])
                expected = height * width * channels * 4
                if len(raw) != expected:
                    entry[name] = {"status": "size_mismatch"}
                    continue
                points = np.frombuffer(raw, dtype="<f4").reshape(-1, channels)[:, :3]
                points = points[np.isfinite(points).all(axis=1)]
                if not len(points):
                    entry[name] = {"valid_points": 0}
                    continue
                z = points[:, 2]
                entry[name] = {
                    "valid_points": int(len(points)),
                    "z_min_m": round(float(z.min()), 4),
                    "z_max_m": round(float(z.max()), 4),
                    "z_range_m": round(float(np.ptp(z)), 4),
                    "z_p5_p95_m": round(float(np.percentile(z, 95) - np.percentile(z, 5)), 4),
                    "x_range_m": round(float(np.ptp(points[:, 0])), 3),
                    "y_range_m": round(float(np.ptp(points[:, 1])), 3),
                }
            samples.append(entry)
        report["sampled_stored_grids"] = samples
        report["status"] = "ok"
    except Exception as e:
        report["status"] = "analysis_error"
        report["error"] = f"{type(e).__name__}: {e}"
    return report


class Trial:
    def __init__(self, a, output: Path):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
        from rtabmap_msgs.msg import Info, MapGraph
        from sensor_msgs.msg import PointCloud2

        self.rclpy = rclpy
        self.Twist = Twist
        self.a = a
        self.output = output
        self.started = time.monotonic()
        self.current_stage = "startup"
        self.latest_odom = None
        self.last_odom_wall = None
        self.odom_count = 0
        self.info_count = 0
        self.graph_count = 0
        self.last_graph = None
        self.cloud_count = 0
        self.last_cloud = None
        self.command_x = 0.0
        self.command_w = 0.0
        self.start_pose = None
        self.last_pose = None
        self.distance_total = 0.0
        self.segments = []
        self.proc = None
        self._launch_log = None
        self._writer = None
        self._csv = None
        self._last_csv_wall = 0.0
        self.event_file = (output / "events.jsonl").open("w", encoding="utf-8", buffering=1)
        self.odom_file = (output / "odom.csv").open("w", newline="", encoding="utf-8")
        self._csv = csv.writer(self.odom_file)
        self._csv.writerow([
            "wall_utc", "elapsed_wall_s", "odom_stamp_s", "stage", "x_m", "y_m", "z_m",
            "yaw_rad", "vx_mps", "vy_mps", "wz_radps", "cmd_vx_mps", "cmd_wz_radps"
        ])

        rclpy.init()
        self.node = rclpy.create_node(NODE_NAME)
        self.node.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, qos_profile_sensor_data)
        self.node.create_subscription(Info, INFO_TOPIC, self._info_cb, qos_profile_sensor_data)
        transient = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.node.create_subscription(MapGraph, GRAPH_TOPIC, self._graph_cb, transient)
        self.node.create_subscription(PointCloud2, CLOUD_TOPIC, self._cloud_cb, transient)
        self.pub = None

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = get_yaw(msg.pose.pose.orientation)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = (float(p.x), float(p.y), float(p.z), yaw)
        if self.last_pose is not None:
            self.distance_total += math.hypot(pose[0] - self.last_pose[0],
                                              pose[1] - self.last_pose[1])
        self.last_pose = pose
        self.latest_odom = pose
        self.last_odom_wall = time.monotonic()
        self.odom_count += 1
        if self.start_pose is None:
            self.start_pose = pose
        # Reduce logging overhead while keeping time-aligned odometry evidence.
        now = time.monotonic()
        if now - self._last_csv_wall >= 0.05:
            self._last_csv_wall = now
            v = msg.twist.twist.linear
            w = msg.twist.twist.angular
            self._csv.writerow([
                utc_now(), round(now - self.started, 4), round(stamp, 7),
                self.current_stage, *[round(x, 7) for x in pose],
                round(float(v.x), 7), round(float(v.y), 7), round(float(w.z), 7),
                round(self.command_x, 5), round(self.command_w, 5)
            ])

    def _info_cb(self, _msg):
        self.info_count += 1

    def _graph_cb(self, msg):
        self.graph_count += 1
        self.last_graph = {
            "frame": msg.header.frame_id,
            "poses": len(msg.poses), "links": len(msg.links)
        }

    def _cloud_cb(self, msg):
        self.cloud_count += 1
        self.last_cloud = {
            "frame": msg.header.frame_id,
            "points": int(msg.width * msg.height)
        }

    def event(self, event_name, **kwargs):
        row = {"utc": utc_now(), "elapsed_wall_s": round(time.monotonic() - self.started, 3),
               "event": event_name, **kwargs}
        self.event_file.write(json.dumps(row) + "\n")
        print(f"[ECHO] {event_name}: {kwargs}", flush=True)

    def spin(self, delay=0.04):
        self.rclpy.spin_once(self.node, timeout_sec=delay)

    def check(self, require_odom=True):
        if time.monotonic() - self.started > self.a.max_wall_s:
            raise RuntimeError(f"Trial exceeded maximum wall time ({self.a.max_wall_s}s)")
        db = self.output / "rtabmap.db"
        if format_mib(db) > self.a.max_db_mib:
            raise RuntimeError(f"Database exceeded {self.a.max_db_mib} MiB cap")
        if self.proc is not None and self.proc.poll() is not None:
            raise RuntimeError(f"RTAB-Map launch exited unexpectedly: {self.proc.returncode}")
        if require_odom and (self.latest_odom is None or
                self.last_odom_wall is None or time.monotonic() - self.last_odom_wall > 1.0):
            raise RuntimeError("Odometry missing/stale (>1 sec)")
        if self.pub is not None and self.node.count_publishers(CMD_TOPIC) > 1:
            raise RuntimeError("A second /cmd_vel publisher appeared; stopping")

    def publish(self, vx=0.0, wz=0.0):
        if self.pub is None:
            return
        msg = self.Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self.command_x, self.command_w = vx, wz
        self.pub.publish(msg)

    def zero(self, duration=0.9):
        if self.pub is None:
            return
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.publish(0, 0)
            self.spin(0.05)

    def start_mapping(self, launch: Path):
        if self.node.count_publishers(CMD_TOPIC) > 0:
            raise RuntimeError("/cmd_vel already has publisher(s); stop Nav2/other drivers")
        if self.node.get_publishers_info_by_topic(REG_TOPIC):
            raise RuntimeError("Registered depth already has a publisher; S02 preflight not clean")
        matches = is_s02_process_present()
        if matches:
            raise RuntimeError("Other registration / RTAB-Map processes found:\n" + "\n".join(matches))
        env = os.environ.copy()
        env["ROS_LOG_DIR"] = str(self.output / "ros_logs")
        (self.output / "ros_logs").mkdir(exist_ok=True)
        self._launch_log = (self.output / "rtabmap_launch.log").open("w", encoding="utf-8")
        cmd = ["ros2", "launch", str(launch),
               f"database_path:={self.output / 'rtabmap.db'}"]
        self.event("rtabmap_launch", command=cmd)
        self.proc = subprocess.Popen(cmd, env=env,
                                     stdout=self._launch_log,
                                     stderr=subprocess.STDOUT,
                                     start_new_session=True)
        self.current_stage = "warmup"
        deadline = time.monotonic() + self.a.warmup_s
        while time.monotonic() < deadline:
            self.spin()
            self.check(require_odom=False)
            if self.info_count >= 3 and self.odom_count >= 10:
                self.event("mapping_ready", info_messages=self.info_count,
                           odom_messages=self.odom_count)
                return
        raise RuntimeError("RTAB-Map did not produce three info messages in warmup period")

    def run_segment(self, name: str, kind: str, target: float):
        self.current_stage = name
        self.check()
        start = self.latest_odom
        start_time = time.monotonic()
        if kind == "forward":
            max_time = max(7.0, target / self.a.forward_mps * 2.5 + 4)
            velocity = (self.a.forward_mps, 0.0)
        else:
            max_time = max(6.0, math.radians(abs(target)) / self.a.turn_radps * 2.5 + 4)
            velocity = (0.0, math.copysign(self.a.turn_radps, target))
        self.event("segment_start", stage=name, kind=kind,
                   target=target, pose=start, timeout_s=round(max_time, 2))
        reached = False
        progress = 0.0
        try:
            while time.monotonic() - start_time < max_time:
                self.check()
                pose = self.latest_odom
                if kind == "forward":
                    progress = math.hypot(pose[0] - start[0], pose[1] - start[1])
                    reached = progress >= target
                else:
                    delta = math.degrees(wrap_angle(pose[3] - start[3]))
                    progress = delta
                    reached = (delta >= target) if target > 0 else (delta <= target)
                if reached:
                    break
                if kind == "forward":
                    # Maintain the heading recorded at segment start.
                    heading_error = wrap_angle(start[3] - pose[3])
                    correction = max(
                        -0.20,
                        min(0.20, 1.2 * heading_error)
                    )
                    self.publish(self.a.forward_mps, correction)
                else:
                    self.publish(*velocity)
                self.spin(0.04)
        finally:
            self.zero(0.8)
        measured_pose = self.latest_odom
        segment = {
            "name": name, "kind": kind, "target": target,
            "reached": bool(reached), "progress_measured": round(progress, 4),
            "elapsed_wall_s": round(time.monotonic() - start_time, 3),
            "start_pose": start, "end_pose": measured_pose,
        }
        self.segments.append(segment)
        self.event("segment_result", **segment)
        if not reached:
            raise RuntimeError(f"Segment {name} did not reach odometry target")
        self.current_stage = f"settle_after_{name}"
        end = time.monotonic() + 1.4
        while time.monotonic() < end:
            self.check()
            self.publish(0, 0)
            self.spin(0.07)

    def stop_mapping(self):
        if self.proc is None:
            return
        proc = self.proc
        try:
            group_id = os.getpgid(proc.pid)
        except ProcessLookupError:
            group_id = None
        if group_id != proc.pid:
            self.event("shutdown_warning", detail="Process group identity mismatch; not signaling group")
            return
        self.event("request_shutdown", pid=proc.pid, group=group_id)
        if proc.poll() is None:
            try:
                os.killpg(group_id, signal.SIGINT)
            except ProcessLookupError:
                pass
        try:
            proc.wait(timeout=24)
            self.event("launch_stopped", returncode=proc.returncode)
        except subprocess.TimeoutExpired:
            self.event("shutdown_escalation", detail="SIGINT timed out, sending SIGTERM to S02 group")
            try:
                os.killpg(group_id, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=12)
                self.event("launch_stopped_after_sigterm", returncode=proc.returncode)
            except subprocess.TimeoutExpired:
                self.event("shutdown_warning", detail="S02 process still alive; inspect manually, do not restart")
        if self._launch_log is not None:
            self._launch_log.flush()

    def close(self):
        try:
            self.zero(0.8)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        try:
            self.rclpy.shutdown()
        except Exception:
            pass
        try:
            self.odom_file.flush()
            self.odom_file.close()
            self.event_file.close()
        except Exception:
            pass
        if self._launch_log is not None:
            self._launch_log.close()

    def metrics(self):
        net = None
        yaw_delta = None
        if self.start_pose and self.latest_odom:
            net = round(math.hypot(self.latest_odom[0]-self.start_pose[0],
                                   self.latest_odom[1]-self.start_pose[1]), 4)
            yaw_delta = round(math.degrees(wrap_angle(self.latest_odom[3]-self.start_pose[3])), 3)
        return {
            "odom_samples": self.odom_count,
            "odom_trajectory_length_m_approx": round(self.distance_total, 4),
            "odom_net_xy_displacement_m": net,
            "odom_net_yaw_delta_deg": yaw_delta,
            "rtabmap_info_messages": self.info_count,
            "map_graph_messages": self.graph_count,
            "last_graph": self.last_graph,
            "cloud_map_messages": self.cloud_count,
            "last_cloud": self.last_cloud,
            "segments": self.segments,
        }


def main() -> int:
    a = parse_args()
    if a.analyze:
        print(json.dumps(analyze_db(a.analyze.expanduser().resolve()), indent=2))
        return 0
    validate_options(a)
    repo = a.repo.expanduser().resolve()
    launch = repo / "launch" / "echo_s02_front_mapping_range10.launch.xml"
    print("=== ECHO S02 MULTIVIEW MAPPING TRIAL ===")
    print("Launcher:", launch)
    print("Route:")
    for name, kind, value in plan(a):
        print(f"  {name:18s} {kind:7s} {value:+.2f} {'m' if kind == 'forward' else 'deg'}")
    print("Maximum database size:", a.max_db_mib, "MiB")
    print("Maximum wall time:", a.max_wall_s, "s")
    print("NOTE: NO COLLISION AVOIDANCE. Only run in a visually clear aisle.")
    if not a.execute:
        print("\nPREVIEW ONLY. Append --execute to start mapping and move the robot.")
        return 0
    if not launch.is_file():
        print(f"[ERROR] Required launch file not found: {launch}", file=sys.stderr)
        return 2
    if "--Grid/RangeMax 10.0" not in launch.read_text():
        print("[ERROR] Expected verified range-10 experimental launcher configuration", file=sys.stderr)
        return 2
    if "ROS_DISTRO" not in os.environ:
        print("[ERROR] Source /opt/ros/jazzy/setup.bash first", file=sys.stderr)
        return 2
    a.output_root = a.output_root.expanduser()
    a.output_root.mkdir(parents=True, exist_ok=True)
    run_dir = a.output_root / dt.datetime.now().strftime("trial_%Y%m%d_%H%M%S")
    run_dir.mkdir(exist_ok=False)
    print("Output directory:", run_dir, flush=True)
    launch_sha256 = hashlib.sha256(launch.read_bytes()).hexdigest()
    git_head = None
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=repo, text=True,
            capture_output=True, timeout=3
        )
        if result.returncode == 0:
            git_head = result.stdout.strip()
    except Exception:
        pass
    trial = None
    status = "not_started"
    error = None
    try:
        trial = Trial(a, run_dir)
        trial.event("trial_start", launch=str(launch),
                    leg_m=a.leg_m, forward_mps=a.forward_mps,
                    yaw_deg=a.yaw_deg)
        trial.start_mapping(launch)
        # Re-check for other cmd_vel owners; creating our own publisher is deliberate.
        if trial.node.count_publishers(CMD_TOPIC):
            raise RuntimeError("External /cmd_vel publisher appeared during RTAB startup")
        trial.pub = trial.node.create_publisher(trial.Twist, CMD_TOPIC, 10)
        trial.zero(0.7)
        for name, kind, val in plan(a):
            trial.run_segment(name, kind, val)
        trial.current_stage = "post_route_observe"
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            trial.check()
            trial.publish(0, 0)
            trial.spin(0.07)
        status = "completed"
    except KeyboardInterrupt:
        status = "interrupted"
        error = "KeyboardInterrupt"
        print("\n[ECHO] Interrupted: stopping robot and RTAB-Map", file=sys.stderr)
    except Exception as exc:
        status = "aborted"
        error = f"{type(exc).__name__}: {exc}"
        print("[ECHO ERROR]", error, file=sys.stderr, flush=True)
        traceback.print_exc()
    finally:
        metrics = {}
        if trial is not None:
            try:
                trial.zero(1.0)
                trial.stop_mapping()
                metrics = trial.metrics()
            finally:
                trial.close()
        report = {
            "trial_status": status,
            "error": error,
            "finished_utc": utc_now(),
            "output_dir": str(run_dir),
            "config": {
                "launch": str(launch),
                "launch_sha256": launch_sha256,
                "git_head": git_head,
                "grid_range_max_m": 10.0,
                "leg_m": a.leg_m,
                "yaw_deg": a.yaw_deg,
                "max_db_mib": a.max_db_mib,
                "max_wall_s": a.max_wall_s,
                "sensor": "simulated front D455",
                "pose_reference": "ROS odometry (NOT independent ground truth)",
            },
            "ros_metrics": metrics,
            "database": analyze_db(run_dir / "rtabmap.db"),
            "remaining_s02_processes": is_s02_process_present(),
        }
        (run_dir / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        print("\n=== ECHO S02 RESULT ===")
        print("Status:", status)
        print("Output:", run_dir)
        print("DB size:", report["database"].get("size_mib"), "MiB")
        print("RTAB info messages:", metrics.get("rtabmap_info_messages", 0))
        print("Approx odom trajectory length:",
              metrics.get("odom_trajectory_length_m_approx"), "m")
        print("DB nodes:", report["database"].get("node_count"))
        print("DB graph links:", report["database"].get("link_count"))
        grids = report["database"].get("sampled_stored_grids", [])
        for item in grids[:3]:
            print("Grid node", item["node_id"],
                  "obstacle Z range:", item.get("obstacle", {}).get("z_range_m"), "m",
                  "XY extents:", item.get("obstacle", {}).get("x_range_m"),
                  item.get("obstacle", {}).get("y_range_m"))
        print("Files: summary.json, odom.csv, events.jsonl, rtabmap_launch.log, rtabmap.db")
        if report["remaining_s02_processes"]:
            print("[WARNING] S02 processes still detected; do NOT start another trial:")
            for proc_line in report["remaining_s02_processes"]:
                print("  ", proc_line)
        else:
            print("[PASS] No leftover S02 RTAB-Map processes detected")
    return 0 if status == "completed" else 1


if __name__ == "__main__":
    sys.exit(main())
