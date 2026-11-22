#!/usr/bin/env python3
"""ECHO S03-D2A: operator-supervised, odometry-bounded 0.10 m motion probe.

Does not start mapping or launch Isaac Sim. Does not provide obstacle avoidance.
Without --execute, this script is a read-only preview.
"""

import argparse
import math
import signal
import sys
import time


TARGET_M = 0.10
SPEED_MPS = 0.04
MAX_WALL_S = 15.0
MAX_ODOM_AGE_S = 0.8
MAX_HEADING_CHANGE_DEG = 10.0


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument("--execute", action="store_true", help="Enable the 10 cm motion")
    execute = args.parse_args().execute

    print("=== ECHO S03-D2A MOTION CALIBRATION ===", flush=True)
    print(f"Command: forward {TARGET_M:.2f} m at {SPEED_MPS:.3f} m/s", flush=True)
    print(f"Wall timeout: {MAX_WALL_S:g} s; stale-odom limit: {MAX_ODOM_AGE_S:g} s", flush=True)
    print("No obstacle avoidance. Visually verify >1.5 m clearance ahead", flush=True)
    print("and no approaching NPC/human before enabling motion.", flush=True)
    if not execute:
        print("PREVIEW ONLY. No ROS publisher was created. To move, add --execute.")
        return 0

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init()
    node = rclpy.create_node("echo_s03_d2a_motion_probe")
    latest = {"pose": None, "wall": None, "frame": None, "child": None, "count": 0}
    stop = {"requested": False}
    publisher = None
    finished = False

    def on_signal(_signum, _frame):
        stop["requested"] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def on_odom(msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        latest.update(pose=(float(p.x), float(p.y), yaw(q)),
                      wall=time.monotonic(), frame=msg.header.frame_id,
                      child=msg.child_frame_id, count=latest["count"] + 1)

    node.create_subscription(Odometry, "/chassis/odom", on_odom,
                             qos_profile_sensor_data)

    try:
        # Wait for ROS graph discovery and healthy odometry.
        deadline = time.monotonic() + 8.0
        stable_since = None
        while time.monotonic() < deadline and not stop["requested"]:
            rclpy.spin_once(node, timeout_sec=0.1)
            publishers = node.count_publishers("/cmd_vel")
            subscribers = node.count_subscribers("/cmd_vel")
            if publishers:
                raise RuntimeError("Another /cmd_vel publisher exists. Refusing to move.")
            odom_ready = (latest["count"] >= 6 and latest["wall"] is not None
                          and time.monotonic() - latest["wall"] <= MAX_ODOM_AGE_S)
            if subscribers == 1 and odom_ready:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since >= 0.5:
                    break
            else:
                stable_since = None
        if stop["requested"]:
            raise RuntimeError("Operator stop requested during preflight.")
        if stable_since is None or time.monotonic() - stable_since < 0.5:
            raise RuntimeError(
                f"ROS graph/odometry not ready within 8 s: "
                f"publishers={node.count_publishers('/cmd_vel')}, "
                f"subscribers={node.count_subscribers('/cmd_vel')}, "
                f"odom_samples={latest['count']}")
        if time.monotonic() - latest["wall"] > MAX_ODOM_AGE_S:
            raise RuntimeError("Odometry is stale at startup.")
        print(f"Odometry ready: {latest['frame']} -> {latest['child']}, "
              f"samples={latest['count']}", flush=True)
        print("Motion starts now. Ctrl+C requests immediate zero velocity.", flush=True)

        publisher = node.create_publisher(Twist, "/cmd_vel", 10)
        start = latest["pose"]
        start_wall = time.monotonic()
        last_report = start_wall
        while True:
            now = time.monotonic()
            if stop["requested"]:
                raise RuntimeError("Operator stop requested")
            if now - start_wall > MAX_WALL_S:
                raise RuntimeError("Wall-time safety limit exceeded")
            if now - latest["wall"] > MAX_ODOM_AGE_S:
                raise RuntimeError("Odometry became stale; stopping")
            if node.count_publishers("/cmd_vel") != 1:
                raise RuntimeError("Another /cmd_vel publisher appeared; stopping")
            if node.count_subscribers("/cmd_vel") != 1:
                raise RuntimeError("Expected /cmd_vel subscriber disappeared or changed; stopping")
            x, y, a = latest["pose"]
            displacement = math.hypot(x - start[0], y - start[1])
            heading_error = math.degrees(abs(angle_delta(a, start[2])))
            if heading_error > MAX_HEADING_CHANGE_DEG:
                raise RuntimeError(f"Heading changed {heading_error:.1f} deg; stopping")
            if displacement >= TARGET_M:
                print(f"TARGET REACHED: odom displacement={displacement:.3f} m; "
                      f"heading change={heading_error:.2f} deg", flush=True)
                finished = True
                break
            command = Twist()
            command.linear.x = SPEED_MPS
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.05)
            if now - last_report > 1.0:
                print(f"Progress: {displacement:.3f}/{TARGET_M:.3f} m", flush=True)
                last_report = now
    except Exception as error:
        print(f"ABORT: {error}", file=sys.stderr, flush=True)
    finally:
        if publisher is not None and rclpy.ok():
            zero = Twist()
            for _ in range(20):
                publisher.publish(zero)
                rclpy.spin_once(node, timeout_sec=0.05)
            print("Zero velocity published for ~1 s.", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if finished else 1


if __name__ == "__main__":
    sys.exit(main())
